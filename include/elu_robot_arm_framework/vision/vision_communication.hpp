/**
 * @file vision_communication.hpp
 * @brief 机器人视觉系统TCP通信接口
 * @author ELU Robotics Team
 * @date 2025
 */

#ifndef ELU_ROBOT_ARM_FRAMEWORK__VISION_COMMUNICATION_HPP_
#define ELU_ROBOT_ARM_FRAMEWORK__VISION_COMMUNICATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "elu_robot_arm_framework/interfaces/robot_arm_interface.hpp"
#include "elu_robot_arm_framework/msg/vision_result.hpp"
#include "elu_robot_arm_framework/srv/trigger_vision.hpp"

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

namespace elu_robot_arm_framework
{

/**
 * @brief 视觉检测结果结构
 */
struct VisionDetectionResult {
    int code;                           ///< 结果代码
    std::vector<geometry_msgs::msg::Pose> poses;  ///< 检测到的位姿列表
    std::string error_message;          ///< 错误信息
    std::chrono::system_clock::time_point timestamp;  ///< 时间戳
    
    VisionDetectionResult() : code(0) {
        timestamp = std::chrono::system_clock::now();
    }
};

/**
 * @brief TCP通信状态枚举
 */
enum class TcpConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR
};

/**
 * @brief 视觉通信模块
 * 
 * 实现机器人与视觉系统的TCP通信，包括：
 * - TCP客户端连接管理
 * - 位姿查询响应
 * - 拍照指令发送
 * - 视觉结果接收处理
 */
class VisionCommunication : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param robot_adapter 机器人适配器指针
     */
    explicit VisionCommunication(std::shared_ptr<RobotArmInterface> robot_adapter,
                                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    virtual ~VisionCommunication();

    // ==================== 公共接口 ====================
    
    /**
     * @brief 连接到视觉服务器
     * @param server_ip 服务器IP地址
     * @param server_port 服务器端口
     * @return 是否连接成功
     */
    bool connectToVisionServer(const std::string& server_ip, int server_port);
    
    /**
     * @brief 断开连接
     */
    void disconnect();
    
    /**
     * @brief 检查连接状态
     * @return 是否已连接
     */
    bool isConnected() const;
    
    /**
     * @brief 获取连接状态
     * @return 连接状态枚举
     */
    TcpConnectionState getConnectionState() const;
    
    /**
     * @brief 触发拍照并获取检测结果
     * @param station_id 工位号
     * @param timeout_sec 超时时间（秒）
     * @return 检测结果
     */
    VisionDetectionResult triggerCapture(int station_id, double timeout_sec = 10.0);
    
    /**
     * @brief 异步触发拍照
     * @param station_id 工位号
     * @return 是否发送成功
     */
    bool triggerCaptureAsync(int station_id);
    
    /**
     * @brief 获取最新的检测结果
     * @return 最新检测结果
     */
    VisionDetectionResult getLatestResult();
    
    /**
     * @brief 设置自动重连
     * @param enable 是否启用自动重连
     * @param interval_sec 重连间隔（秒）
     */
    void setAutoReconnect(bool enable, double interval_sec = 5.0);

protected:
    // ==================== TCP通信核心 ====================
    
    /**
     * @brief 初始化网络库（Windows需要）
     */
    bool initializeNetwork();
    
    /**
     * @brief 清理网络库
     */
    void cleanupNetwork();
    
    /**
     * @brief 创建TCP客户端socket
     */
    bool createSocket();
    
    /**
     * @brief 连接到服务器
     */
    bool connectSocket(const std::string& server_ip, int server_port);
    
    /**
     * @brief 关闭socket连接
     */
    void closeSocket();
    
    /**
     * @brief 发送数据
     * @param data 要发送的数据
     * @return 是否发送成功
     */
    bool sendData(const std::string& data);
    
    /**
     * @brief 接收数据
     * @param buffer 接收缓冲区
     * @param max_size 最大接收大小
     * @return 实际接收的字节数，-1表示错误
     */
    int receiveData(char* buffer, int max_size);
    
    // ==================== 协议处理 ====================
    
    /**
     * @brief 处理接收到的消息
     * @param message 接收到的消息
     */
    void processReceivedMessage(const std::string& message);
    
    /**
     * @brief 处理位姿查询请求
     * @return 响应消息
     */
    std::string handlePoseQuery();
    
    /**
     * @brief 处理视觉检测结果
     * @param message 结果消息
     */
    void handleVisionResult(const std::string& message);
    
    /**
     * @brief 发送拍照指令
     * @param station_id 工位号
     * @return 是否发送成功
     */
    bool sendCaptureCommand(int station_id);
    
    // ==================== 数据转换 ====================
    
    /**
     * @brief 将机器人位姿转换为通信格式
     * @param pose ROS位姿
     * @return 格式化字符串 "x,y,z,a,b,c"
     */
    std::string poseToString(const geometry_msgs::msg::Pose& pose);
    
    /**
     * @brief 从通信格式解析位姿
     * @param pose_str 位姿字符串 "x,y,z,a,b,c"
     * @return ROS位姿
     */
    geometry_msgs::msg::Pose stringToPose(const std::string& pose_str);
    
    /**
     * @brief 四元数转欧拉角 (ZYX顺序)
     * @param qx, qy, qz, qw 四元数分量
     * @param roll, pitch, yaw 输出欧拉角（弧度）
     */
    void quaternionToEuler(double qx, double qy, double qz, double qw,
                          double& roll, double& pitch, double& yaw);
    
    /**
     * @brief 欧拉角转四元数 (ZYX顺序)
     * @param roll, pitch, yaw 欧拉角（弧度）
     * @param qx, qy, qz, qw 输出四元数分量
     */
    void eulerToQuaternion(double roll, double pitch, double yaw,
                          double& qx, double& qy, double& qz, double& qw);
    
    // ==================== 线程管理 ====================
    
    /**
     * @brief 通信线程主函数
     */
    void communicationThread();
    
    /**
     * @brief 重连线程主函数
     */
    void reconnectionThread();
    
    /**
     * @brief 心跳线程主函数
     */
    void heartbeatThread();

private:
    // ==================== 成员变量 ====================
    
    // 机器人接口
    std::shared_ptr<RobotArmInterface> robot_adapter_;
    
    // 网络相关
#ifdef _WIN32
    SOCKET socket_fd_;
    WSADATA wsa_data_;
#else
    int socket_fd_;
#endif
    std::string server_ip_;
    int server_port_;
    std::atomic<TcpConnectionState> connection_state_;
    
    // 线程管理
    std::thread comm_thread_;
    std::thread reconnect_thread_;
    std::thread heartbeat_thread_;
    std::atomic<bool> thread_running_;
    std::atomic<bool> stop_requested_;
    
    // 同步原语
    mutable std::mutex state_mutex_;
    mutable std::mutex result_mutex_;
    std::condition_variable result_condition_;
    
    // 通信配置
    std::atomic<bool> auto_reconnect_enabled_;
    std::atomic<double> reconnect_interval_;
    static constexpr int SOCKET_TIMEOUT_MS = 5000;
    static constexpr int RECV_BUFFER_SIZE = 4096;
    static constexpr double HEARTBEAT_INTERVAL = 30.0;  // 30秒心跳
    
    // 结果管理
    std::queue<VisionDetectionResult> result_queue_;
    VisionDetectionResult latest_result_;
    std::atomic<bool> waiting_for_result_;
    
    // ROS2 接口
    rclcpp::Publisher<msg::VisionResult>::SharedPtr vision_result_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr connection_status_pub_;
    rclcpp::Service<srv::TriggerVision>::SharedPtr trigger_vision_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 服务回调函数
    void triggerVisionCallback(
        const std::shared_ptr<srv::TriggerVision::Request> request,
        std::shared_ptr<srv::TriggerVision::Response> response);
    void connectServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void disconnectServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void statusTimerCallback();
    
    // 工具函数
    std::vector<std::string> splitString(const std::string& str, char delimiter);
    std::string formatDouble(double value, int precision = 3);
    bool isValidMessage(const std::string& message);
    void logConnectionState(TcpConnectionState state);
};

} // namespace elu_robot_arm_framework

#endif // ELU_ROBOT_ARM_FRAMEWORK__VISION_COMMUNICATION_HPP_