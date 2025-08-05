/**
 * @file vision_communication.cpp
 * @brief 机器人视觉系统TCP通信实现
 */

#include "elu_robot_arm_framework/vision/vision_communication.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace elu_robot_arm_framework
{

VisionCommunication::VisionCommunication(std::shared_ptr<RobotArmInterface> robot_adapter,
                                        const rclcpp::NodeOptions& options)
    : Node("vision_communication", options)
    , robot_adapter_(robot_adapter)
#ifdef _WIN32
    , socket_fd_(INVALID_SOCKET)
#else
    , socket_fd_(-1)
#endif
    , server_port_(0)
    , connection_state_(TcpConnectionState::DISCONNECTED)
    , thread_running_(false)
    , stop_requested_(false)
    , auto_reconnect_enabled_(true)
    , reconnect_interval_(5.0)
    , waiting_for_result_(false)
{
    // 声明参数
    this->declare_parameter("server_ip", "192.168.1.100");
    this->declare_parameter("server_port", 8080);
    this->declare_parameter("auto_reconnect", true);
    this->declare_parameter("reconnect_interval", 5.0);
    this->declare_parameter("auto_connect", false);

    // 获取参数
    server_ip_ = this->get_parameter("server_ip").as_string();
    server_port_ = this->get_parameter("server_port").as_int();
    auto_reconnect_enabled_ = this->get_parameter("auto_reconnect").as_bool();
    reconnect_interval_ = this->get_parameter("reconnect_interval").as_double();
    bool auto_connect = this->get_parameter("auto_connect").as_bool();

    // 初始化ROS2接口
    vision_result_pub_ = this->create_publisher<msg::VisionResult>("vision_result", 10);
    connection_status_pub_ = this->create_publisher<std_msgs::msg::String>("vision_connection_status", 10);
    
    trigger_vision_service_ = this->create_service<srv::TriggerVision>(
        "trigger_vision",
        std::bind(&VisionCommunication::triggerVisionCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    connect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "connect_vision",
        std::bind(&VisionCommunication::connectServiceCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "disconnect_vision",
        std::bind(&VisionCommunication::disconnectServiceCallback, this,
                 std::placeholders::_1, std::placeholders::_2));

    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&VisionCommunication::statusTimerCallback, this));

    // 初始化网络
    if (!initializeNetwork()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize network");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Vision Communication Node initialized");
    RCLCPP_INFO(this->get_logger(), "Target server: %s:%d", server_ip_.c_str(), server_port_);

    // 自动连接
    if (auto_connect) {
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            connectToVisionServer(server_ip_, server_port_);
        }).detach();
    }
}

VisionCommunication::~VisionCommunication()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Vision Communication");

    // 停止所有线程
    stop_requested_ = true;
    thread_running_ = false;

    // 断开连接
    disconnect();

    // 等待线程结束
    if (comm_thread_.joinable()) {
        comm_thread_.join();
    }
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }

    // 清理网络
    cleanupNetwork();
}

// ==================== 公共接口实现 ====================

bool VisionCommunication::connectToVisionServer(const std::string& server_ip, int server_port)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (connection_state_ == TcpConnectionState::CONNECTED) {
        RCLCPP_WARN(this->get_logger(), "Already connected to vision server");
        return true;
    }

    server_ip_ = server_ip;
    server_port_ = server_port;

    RCLCPP_INFO(this->get_logger(), "Connecting to vision server %s:%d", server_ip.c_str(), server_port);

    connection_state_ = TcpConnectionState::CONNECTING;
    logConnectionState(connection_state_);

    // 创建socket并连接
    if (!createSocket()) {
        connection_state_ = TcpConnectionState::ERROR;
        logConnectionState(connection_state_);
        return false;
    }

    if (!connectSocket(server_ip, server_port)) {
        closeSocket();
        connection_state_ = TcpConnectionState::ERROR;
        logConnectionState(connection_state_);
        return false;
    }

    connection_state_ = TcpConnectionState::CONNECTED;
    logConnectionState(connection_state_);

    // 启动通信线程
    if (!thread_running_) {
        thread_running_ = true;
        stop_requested_ = false;
        
        comm_thread_ = std::thread(&VisionCommunication::communicationThread, this);
        
        if (auto_reconnect_enabled_) {
            reconnect_thread_ = std::thread(&VisionCommunication::reconnectionThread, this);
        }
        
        heartbeat_thread_ = std::thread(&VisionCommunication::heartbeatThread, this);
    }

    RCLCPP_INFO(this->get_logger(), "✓ Successfully connected to vision server");
    return true;
}

void VisionCommunication::disconnect()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (connection_state_ == TcpConnectionState::DISCONNECTED) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Disconnecting from vision server");

    // 关闭socket
    closeSocket();
    connection_state_ = TcpConnectionState::DISCONNECTED;
    logConnectionState(connection_state_);

    RCLCPP_INFO(this->get_logger(), "✓ Disconnected from vision server");
}

bool VisionCommunication::isConnected() const
{
    return connection_state_ == TcpConnectionState::CONNECTED;
}

TcpConnectionState VisionCommunication::getConnectionState() const
{
    return connection_state_;
}

VisionDetectionResult VisionCommunication::triggerCapture(int station_id, double timeout_sec)
{
    if (!isConnected()) {
        VisionDetectionResult result;
        result.code = -2;
        result.error_message = "Not connected to vision server";
        return result;
    }

    // 发送拍照指令
    if (!sendCaptureCommand(station_id)) {
        VisionDetectionResult result;
        result.code = -2;
        result.error_message = "Failed to send capture command";
        return result;
    }

    // 等待结果
    std::unique_lock<std::mutex> lock(result_mutex_);
    waiting_for_result_ = true;

    bool received = result_condition_.wait_for(lock, 
        std::chrono::duration<double>(timeout_sec),
        [this] { return !waiting_for_result_; });

    if (!received) {
        waiting_for_result_ = false;
        VisionDetectionResult result;
        result.code = -1;
        result.error_message = "Timeout waiting for vision result";
        return result;
    }

    return latest_result_;
}

bool VisionCommunication::triggerCaptureAsync(int station_id)
{
    return sendCaptureCommand(station_id);
}

VisionDetectionResult VisionCommunication::getLatestResult()
{
    std::lock_guard<std::mutex> lock(result_mutex_);
    return latest_result_;
}

void VisionCommunication::setAutoReconnect(bool enable, double interval_sec)
{
    auto_reconnect_enabled_ = enable;
    reconnect_interval_ = interval_sec;
    
    RCLCPP_INFO(this->get_logger(), "Auto reconnect %s, interval: %.1fs", 
                enable ? "enabled" : "disabled", interval_sec);
}

// ==================== TCP通信核心实现 ====================

bool VisionCommunication::initializeNetwork()
{
#ifdef _WIN32
    int result = WSAStartup(MAKEWORD(2, 2), &wsa_data_);
    if (result != 0) {
        RCLCPP_ERROR(this->get_logger(), "WSAStartup failed: %d", result);
        return false;
    }
#endif
    return true;
}

void VisionCommunication::cleanupNetwork()
{
#ifdef _WIN32
    WSACleanup();
#endif
}

bool VisionCommunication::createSocket()
{
#ifdef _WIN32
    socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_ == INVALID_SOCKET) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %d", WSAGetLastError());
        return false;
    }
#else
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
        return false;
    }
#endif

    // 设置socket选项
#ifdef _WIN32
    DWORD timeout = SOCKET_TIMEOUT_MS;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
#else
    struct timeval tv;
    tv.tv_sec = SOCKET_TIMEOUT_MS / 1000;
    tv.tv_usec = (SOCKET_TIMEOUT_MS % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    // 设置地址重用
    int opt = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

    return true;
}

bool VisionCommunication::connectSocket(const std::string& server_ip, int server_port)
{
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);

#ifdef _WIN32
    if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid server IP address: %s", server_ip.c_str());
        return false;
    }
#else
    if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid server IP address: %s", server_ip.c_str());
        return false;
    }
#endif

    int result = connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr));
#ifdef _WIN32
    if (result == SOCKET_ERROR) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to server: %d", WSAGetLastError());
        return false;
    }
#else
    if (result < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to server: %s", strerror(errno));
        return false;
    }
#endif

    return true;
}

void VisionCommunication::closeSocket()
{
#ifdef _WIN32
    if (socket_fd_ != INVALID_SOCKET) {
        closesocket(socket_fd_);
        socket_fd_ = INVALID_SOCKET;
    }
#else
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
#endif
}

bool VisionCommunication::sendData(const std::string& data)
{
    if (!isConnected()) {
        return false;
    }

    std::string message = data;
    if (message.back() != '\n') {
        message += '\n';  // 添加换行符作为消息结束标志
    }

#ifdef _WIN32
    int result = send(socket_fd_, message.c_str(), static_cast<int>(message.length()), 0);
    if (result == SOCKET_ERROR) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send data: %d", WSAGetLastError());
        return false;
    }
#else
    ssize_t result = send(socket_fd_, message.c_str(), message.length(), MSG_NOSIGNAL);
    if (result < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", strerror(errno));
        return false;
    }
#endif

    RCLCPP_DEBUG(this->get_logger(), "Sent: %s", data.c_str());
    return true;
}

int VisionCommunication::receiveData(char* buffer, int max_size)
{
    if (!isConnected()) {
        return -1;
    }

#ifdef _WIN32
    int result = recv(socket_fd_, buffer, max_size - 1, 0);
    if (result == SOCKET_ERROR) {
        int error = WSAGetLastError();
        if (error != WSAETIMEDOUT) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive data: %d", error);
        }
        return -1;
    }
#else
    ssize_t result = recv(socket_fd_, buffer, max_size - 1, 0);
    if (result < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive data: %s", strerror(errno));
        }
        return -1;
    }
#endif

    if (result == 0) {
        // 连接已关闭
        return 0;
    }

    buffer[result] = '\0';
    return static_cast<int>(result);
}

// ==================== 协议处理实现 ====================

void VisionCommunication::processReceivedMessage(const std::string& message)
{
    if (!isValidMessage(message)) {
        RCLCPP_WARN(this->get_logger(), "Received invalid message: %s", message.c_str());
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Received: %s", message.c_str());

    // 检查消息类型
    if (message.find("getRobotPose") == 0) {
        // 位姿查询请求
        std::string response = handlePoseQuery();
        sendData(response);
    } else if (message.find_first_of("0123456789-") == 0) {
        // 可能是视觉检测结果
        handleVisionResult(message);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown message type: %s", message.c_str());
    }
}

std::string VisionCommunication::handlePoseQuery()
{
    if (!robot_adapter_) {
        RCLCPP_ERROR(this->get_logger(), "Robot adapter not available");
        return "p,0.000,0.000,0.000,0.000,0.000,0.000,";
    }

    try {
        // 获取当前机器人位姿
        geometry_msgs::msg::Pose current_pose = robot_adapter_->getCurrentPose();
        
        // 转换为通信格式
        std::string pose_str = poseToString(current_pose);
        
        std::string response = "p," + pose_str;
        
        RCLCPP_DEBUG(this->get_logger(), "Pose query response: %s", response.c_str());
        return response;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in handlePoseQuery: %s", e.what());
        return "p,0.000,0.000,0.000,0.000,0.000,0.000,";
    }
}

void VisionCommunication::handleVisionResult(const std::string& message)
{
    std::vector<std::string> parts = splitString(message, ',');
    
    if (parts.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty vision result message");
        return;
    }

    VisionDetectionResult result;
    
    try {
        result.code = std::stoi(parts[0]);
        
        switch (result.code) {
            case 0:
                result.error_message = "Empty box detected";
                RCLCPP_INFO(this->get_logger(), "Vision result: Empty box");
                break;
                
            case -1:
                result.error_message = "Point cloud detected but no coordinates extracted";
                RCLCPP_INFO(this->get_logger(), "Vision result: Point cloud but no coordinates");
                break;
                
            case -2:
                result.error_message = "Invalid command";
                RCLCPP_WARN(this->get_logger(), "Vision result: Invalid command");
                break;
                
            case -3:
                result.error_message = "Detection mode not enabled";
                RCLCPP_WARN(this->get_logger(), "Vision result: Detection mode not enabled");
                break;
                
            default:
                if (result.code > 0) {
                    // 检测到目标，解析坐标
                    int expected_coords = result.code * 6;  // 每个坐标6个值 (x,y,z,a,b,c)
                    
                    if (parts.size() >= static_cast<size_t>(1 + expected_coords)) {
                        for (int i = 0; i < result.code; ++i) {
                            int start_idx = 1 + i * 6;
                            
                            if (static_cast<size_t>(start_idx + 5) < parts.size()) {
                                double x = std::stod(parts[start_idx]);
                                double y = std::stod(parts[start_idx + 1]);
                                double z = std::stod(parts[start_idx + 2]);
                                double a = std::stod(parts[start_idx + 3]) * M_PI / 180.0;  // 转换为弧度
                                double b = std::stod(parts[start_idx + 4]) * M_PI / 180.0;
                                double c = std::stod(parts[start_idx + 5]) * M_PI / 180.0;
                                
                                geometry_msgs::msg::Pose pose;
                                pose.position.x = x / 1000.0;  // mm转m
                                pose.position.y = y / 1000.0;
                                pose.position.z = z / 1000.0;
                                
                                // ZYX欧拉角转四元数
                                double qx, qy, qz, qw;
                                eulerToQuaternion(a, b, c, qx, qy, qz, qw);
                                pose.orientation.x = qx;
                                pose.orientation.y = qy;
                                pose.orientation.z = qz;
                                pose.orientation.w = qw;
                                
                                result.poses.push_back(pose);
                            }
                        }
                        
                        RCLCPP_INFO(this->get_logger(), "Vision result: %d coordinates detected", result.code);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Vision result: Insufficient coordinate data");
                        result.code = -2;
                        result.error_message = "Insufficient coordinate data";
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Vision result: Unknown code %d", result.code);
                    result.error_message = "Unknown result code";
                }
                break;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception parsing vision result: %s", e.what());
        result.code = -2;
        result.error_message = "Parse error: " + std::string(e.what());
    }

    // 保存结果并通知等待的线程
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        latest_result_ = result;
        result_queue_.push(result);
        
        // 限制队列大小
        while (result_queue_.size() > 10) {
            result_queue_.pop();
        }
        
        if (waiting_for_result_) {
            waiting_for_result_ = false;
            result_condition_.notify_all();
        }
    }

    // 发布ROS消息
    auto msg = std::make_shared<msg::VisionResult>();
    msg->header.stamp = this->now();
    msg->code = result.code;
    msg->error_message = result.error_message;
    msg->poses = result.poses;
    vision_result_pub_->publish(*msg);
}

bool VisionCommunication::sendCaptureCommand(int station_id)
{
    if (!robot_adapter_) {
        RCLCPP_ERROR(this->get_logger(), "Robot adapter not available");
        return false;
    }

    try {
        // 获取当前机器人位姿 (眼在手外时可以为0)
        geometry_msgs::msg::Pose current_pose;
        std::string pose_str = "0.000,0.000,0.000,0.000,0.000,0.000";
        
        // 如果需要发送实际位姿，取消注释下面的代码
        // current_pose = robot_adapter_->getCurrentPose();
        // pose_str = poseToString(current_pose);
        
        std::ostringstream cmd;
        cmd << "d," << pose_str << "," << station_id << ",";
        
        std::string command = cmd.str();
        RCLCPP_INFO(this->get_logger(), "Sending capture command: %s", command.c_str());
        
        return sendData(command);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in sendCaptureCommand: %s", e.what());
        return false;
    }
}

// ==================== 数据转换实现 ====================

std::string VisionCommunication::poseToString(const geometry_msgs::msg::Pose& pose)
{
    // 位置转换为mm
    double x = pose.position.x * 1000.0;
    double y = pose.position.y * 1000.0;
    double z = pose.position.z * 1000.0;
    
    // 四元数转欧拉角 (ZYX顺序)
    double roll, pitch, yaw;
    quaternionToEuler(pose.orientation.x, pose.orientation.y, 
                     pose.orientation.z, pose.orientation.w,
                     roll, pitch, yaw);
    
    // 弧度转角度
    double a = roll * 180.0 / M_PI;
    double b = pitch * 180.0 / M_PI;
    double c = yaw * 180.0 / M_PI;
    
    std::ostringstream oss;
    oss << formatDouble(x) << "," << formatDouble(y) << "," << formatDouble(z) << ","
        << formatDouble(a) << "," << formatDouble(b) << "," << formatDouble(c) << ",";
    
    return oss.str();
}

geometry_msgs::msg::Pose VisionCommunication::stringToPose(const std::string& pose_str)
{
    geometry_msgs::msg::Pose pose;
    std::vector<std::string> parts = splitString(pose_str, ',');
    
    if (parts.size() >= 6) {
        // 位置 (mm转m)
        pose.position.x = std::stod(parts[0]) / 1000.0;
        pose.position.y = std::stod(parts[1]) / 1000.0;
        pose.position.z = std::stod(parts[2]) / 1000.0;
        
        // 姿态 (角度转弧度)
        double a = std::stod(parts[3]) * M_PI / 180.0;
        double b = std::stod(parts[4]) * M_PI / 180.0;
        double c = std::stod(parts[5]) * M_PI / 180.0;
        
        // 欧拉角转四元数
        double qx, qy, qz, qw;
        eulerToQuaternion(a, b, c, qx, qy, qz, qw);
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;
    }
    
    return pose;
}

void VisionCommunication::quaternionToEuler(double qx, double qy, double qz, double qw,
                                           double& roll, double& pitch, double& yaw)
{
    // ZYX欧拉角转换 (内旋顺序)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void VisionCommunication::eulerToQuaternion(double roll, double pitch, double yaw,
                                           double& qx, double& qy, double& qz, double& qw)
{
    // ZYX欧拉角转四元数
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

// ==================== 线程管理实现 ====================

void VisionCommunication::communicationThread()
{
    char buffer[RECV_BUFFER_SIZE];
    std::string message_buffer;

    RCLCPP_INFO(this->get_logger(), "Communication thread started");

    while (thread_running_ && !stop_requested_) {
        if (!isConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        int bytes_received = receiveData(buffer, RECV_BUFFER_SIZE);
        
        if (bytes_received > 0) {
            message_buffer += std::string(buffer, bytes_received);
            
            // 处理完整的消息（以换行符分割）
            size_t pos = 0;
            while ((pos = message_buffer.find('\n')) != std::string::npos) {
                std::string message = message_buffer.substr(0, pos);
                message_buffer.erase(0, pos + 1);
                
                // 移除回车符
                if (!message.empty() && message.back() == '\r') {
                    message.pop_back();
                }
                
                if (!message.empty()) {
                    processReceivedMessage(message);
                }
            }
        } else if (bytes_received == 0) {
            // 连接已关闭
            RCLCPP_WARN(this->get_logger(), "Vision server closed connection");
            connection_state_ = TcpConnectionState::DISCONNECTED;
            logConnectionState(connection_state_);
            closeSocket();
            break;
        }
        // bytes_received < 0 是超时或其他错误，继续循环
    }

    RCLCPP_INFO(this->get_logger(), "Communication thread stopped");
}

void VisionCommunication::reconnectionThread()
{
    RCLCPP_INFO(this->get_logger(), "Reconnection thread started");

    while (thread_running_ && !stop_requested_) {
        if (!auto_reconnect_enabled_ || isConnected()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        if (connection_state_ == TcpConnectionState::DISCONNECTED || 
            connection_state_ == TcpConnectionState::ERROR) {
            
            RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to vision server...");
            
            if (connectToVisionServer(server_ip_, server_port_)) {
                RCLCPP_INFO(this->get_logger(), "✓ Reconnection successful");
            } else {
                RCLCPP_WARN(this->get_logger(), "✗ Reconnection failed, retrying in %.1fs", reconnect_interval_.load());
            }
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_interval_));
    }

    RCLCPP_INFO(this->get_logger(), "Reconnection thread stopped");
}

void VisionCommunication::heartbeatThread()
{
    RCLCPP_INFO(this->get_logger(), "Heartbeat thread started");

    while (thread_running_ && !stop_requested_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(HEARTBEAT_INTERVAL));

        if (isConnected()) {
            // 发送心跳包（获取位姿查询）
            // 注意：实际实现中可能需要定制心跳协议
            RCLCPP_DEBUG(this->get_logger(), "Sending heartbeat");
        }
    }

    RCLCPP_INFO(this->get_logger(), "Heartbeat thread stopped");
}

// ==================== ROS2服务回调实现 ====================

void VisionCommunication::triggerVisionCallback(
    const std::shared_ptr<srv::TriggerVision::Request> request,
    std::shared_ptr<srv::TriggerVision::Response> response)
{
    VisionDetectionResult result = triggerCapture(request->station_id, request->timeout);
    
    response->success = (result.code >= 0);
    response->message = result.error_message;
    response->result_code = result.code;
    response->poses = result.poses;
}

void VisionCommunication::connectServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    
    if (connectToVisionServer(server_ip_, server_port_)) {
        response->success = true;
        response->message = "Connected to vision server successfully";
    } else {
        response->success = false;
        response->message = "Failed to connect to vision server";
    }
}

void VisionCommunication::disconnectServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    
    disconnect();
    response->success = true;
    response->message = "Disconnected from vision server";
}

void VisionCommunication::statusTimerCallback()
{
    // 发布连接状态
    auto status_msg = std::make_shared<std_msgs::msg::String>();
    
    switch (connection_state_) {
        case TcpConnectionState::DISCONNECTED:
            status_msg->data = "DISCONNECTED";
            break;
        case TcpConnectionState::CONNECTING:
            status_msg->data = "CONNECTING";
            break;
        case TcpConnectionState::CONNECTED:
            status_msg->data = "CONNECTED";
            break;
        case TcpConnectionState::ERROR:
            status_msg->data = "ERROR";
            break;
        default:
            status_msg->data = "UNKNOWN";
            break;
    }
    
    connection_status_pub_->publish(*status_msg);
}

// ==================== 工具函数实现 ====================

std::vector<std::string> VisionCommunication::splitString(const std::string& str, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}

std::string VisionCommunication::formatDouble(double value, int precision)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

bool VisionCommunication::isValidMessage(const std::string& message)
{
    return !message.empty() && message.length() < 1024;  // 基本验证
}

void VisionCommunication::logConnectionState(TcpConnectionState state)
{
    std::string state_str;
    switch (state) {
        case TcpConnectionState::DISCONNECTED:
            state_str = "DISCONNECTED";
            break;
        case TcpConnectionState::CONNECTING:
            state_str = "CONNECTING";
            break;
        case TcpConnectionState::CONNECTED:
            state_str = "CONNECTED";
            break;
        case TcpConnectionState::ERROR:
            state_str = "ERROR";
            break;
    }
    
    RCLCPP_INFO(this->get_logger(), "Vision connection state: %s", state_str.c_str());
}

} // namespace elu_robot_arm_framework