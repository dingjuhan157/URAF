/**
 * @file vision_communication_node.cpp
 * @brief 独立的视觉通信节点
 * 
 * 此节点专门负责与视觉系统的TCP通信，可以独立运行
 * 也可以与其他机器人控制节点协同工作
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <csignal>
#include "elu_robot_arm_framework/vision/vision_communication.hpp"
#include "elu_robot_arm_framework/adapters/rokae_adapter.hpp"

/**
 * @brief 独立视觉通信节点类
 * 
 * 该节点可以独立于机器人控制运行，专门处理视觉通信
 * 也可以连接到现有的机器人适配器
 */
class VisionCommunicationNode : public rclcpp::Node
{
public:
    VisionCommunicationNode() : Node("vision_communication_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Vision Communication Node");

        // 声明参数
        this->declare_parameter("use_robot_adapter", false);
        this->declare_parameter("robot_config_file", "config/robots/rokae_cr7_config.yaml");
        this->declare_parameter("vision_server_ip", "192.168.84.1");
        this->declare_parameter("vision_server_port", 8080);
        this->declare_parameter("auto_connect", true);
        this->declare_parameter("auto_reconnect", true);
        this->declare_parameter("reconnect_interval", 5.0);

        // 获取参数
        bool use_robot_adapter = this->get_parameter("use_robot_adapter").as_bool();
        std::string robot_config_file = this->get_parameter("robot_config_file").as_string();
        std::string vision_server_ip = this->get_parameter("vision_server_ip").as_string();
        int vision_server_port = this->get_parameter("vision_server_port").as_int();
        bool auto_connect = this->get_parameter("auto_connect").as_bool();
        bool auto_reconnect = this->get_parameter("auto_reconnect").as_bool();
        double reconnect_interval = this->get_parameter("reconnect_interval").as_double();

        // 可选的机器人适配器
        std::shared_ptr<elu_robot_arm_framework::RobotArmInterface> robot_adapter = nullptr;
        
        if (use_robot_adapter) {
            RCLCPP_INFO(this->get_logger(), "Initializing robot adapter...");
            try {
                auto rokae_adapter = std::make_shared<elu_robot_arm_framework::RokaeAdapter>();
                
                // 尝试连接机器人
                if (rokae_adapter->connect(robot_config_file)) {
                    robot_adapter = rokae_adapter;
                    RCLCPP_INFO(this->get_logger(), "✓ Robot adapter connected successfully");
                } else {
                    RCLCPP_WARN(this->get_logger(), "⚠️ Failed to connect robot adapter, continuing without robot");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception initializing robot adapter: %s", e.what());
                RCLCPP_WARN(this->get_logger(), "Continuing without robot adapter");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Running in vision-only mode (no robot adapter)");
        }

        // 创建视觉通信对象
        rclcpp::NodeOptions vision_options;
        vision_comm_ = std::make_shared<elu_robot_arm_framework::VisionCommunication>(
            robot_adapter, vision_options);

        // 配置自动重连
        vision_comm_->setAutoReconnect(auto_reconnect, reconnect_interval);

        RCLCPP_INFO(this->get_logger(), "Vision Communication Node initialized");
        RCLCPP_INFO(this->get_logger(), "Target server: %s:%d", vision_server_ip.c_str(), vision_server_port);
        RCLCPP_INFO(this->get_logger(), "Robot adapter: %s", use_robot_adapter ? "enabled" : "disabled");

        // 自动连接
        if (auto_connect) {
            connection_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                [this, vision_server_ip, vision_server_port]() {
                    connection_timer_->cancel();
                    
                    RCLCPP_INFO(this->get_logger(), "Auto-connecting to vision server...");
                    if (vision_comm_->connectToVisionServer(vision_server_ip, vision_server_port)) {
                        RCLCPP_INFO(this->get_logger(), "✓ Auto-connection successful");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "✗ Auto-connection failed");
                    }
                });
        }

        // 状态监控定时器
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VisionCommunicationNode::statusCallback, this));

        RCLCPP_INFO(this->get_logger(), "🎯 Vision Communication Node ready");
        printUsageInstructions();
    }

    ~VisionCommunicationNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Vision Communication Node");
        
        if (vision_comm_) {
            vision_comm_->disconnect();
        }
    }

private:
    void statusCallback()
    {
        if (!vision_comm_) return;

        static auto last_state = elu_robot_arm_framework::TcpConnectionState::DISCONNECTED;
        auto current_state = vision_comm_->getConnectionState();

        if (current_state != last_state) {
            std::string state_str;
            switch (current_state) {
                case elu_robot_arm_framework::TcpConnectionState::DISCONNECTED:
                    state_str = "DISCONNECTED";
                    break;
                case elu_robot_arm_framework::TcpConnectionState::CONNECTING:
                    state_str = "CONNECTING";
                    break;
                case elu_robot_arm_framework::TcpConnectionState::CONNECTED:
                    state_str = "CONNECTED ✓";
                    break;
                case elu_robot_arm_framework::TcpConnectionState::ERROR:
                    state_str = "ERROR ✗";
                    break;
            }
            
            RCLCPP_INFO(this->get_logger(), "Vision connection status: %s", state_str.c_str());
            last_state = current_state;
        }
    }

    void printUsageInstructions()
    {
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "📋 === Vision Communication Node Usage ===");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "🔗 Connection Management:");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /connect_vision std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /disconnect_vision std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "🎯 Vision Detection:");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /trigger_vision elu_robot_arm_framework/srv/TriggerVision \"{station_id: 1, timeout: 10.0}\" ");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "📊 Monitoring:");
        RCLCPP_INFO(this->get_logger(), "  ros2 topic echo /vision_result");
        RCLCPP_INFO(this->get_logger(), "  ros2 topic echo /vision_connection_status");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "🔧 Testing:");
        RCLCPP_INFO(this->get_logger(), "  # Test connection");
        RCLCPP_INFO(this->get_logger(), "  ros2 topic echo /vision_connection_status --once");
        RCLCPP_INFO(this->get_logger(), "  # Test detection");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /trigger_vision elu_robot_arm_framework/srv/TriggerVision \"{station_id: 1, timeout: 15.0}\" ");
        RCLCPP_INFO(this->get_logger(), "  # Monitor results");
        RCLCPP_INFO(this->get_logger(), "  ros2 topic echo /vision_result --once");
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), " ");
    }

private:
    std::shared_ptr<elu_robot_arm_framework::VisionCommunication> vision_comm_;
    rclcpp::TimerBase::SharedPtr connection_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

/**
 * @brief 模拟机器人适配器 (用于测试)
 * 
 * 当没有实际机器人时，可以使用此模拟适配器进行测试
 */
class MockRobotAdapter : public elu_robot_arm_framework::RobotArmInterface
{
public:
    MockRobotAdapter() : connected_(false), current_joints_(6, 0.0) {
        // 设置模拟的当前位姿
        current_pose_.position.x = 0.5;
        current_pose_.position.y = 0.0;
        current_pose_.position.z = 0.3;
        current_pose_.orientation.w = 1.0;
    }

    // 连接管理
    bool connect(const std::string& config_file) override {
        (void)config_file;
        connected_ = true;
        return true;
    }
    
    bool disconnect() override {
        connected_ = false;
        return true;
    }
    
    bool isConnected() const override {
        return connected_;
    }

    // 运动控制 (模拟实现)
    bool moveToJoint(const std::vector<double>& joints, double speed_ratio = 1.0) override {
        (void)speed_ratio;
        if (joints.size() == 6) {
            current_joints_ = joints;
            return true;
        }
        return false;
    }
    
    bool moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) override {
        (void)speed_ratio;
        current_pose_ = pose;
        return true;
    }
    
    bool linearMove(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) override {
        return moveToPose(pose, speed_ratio);
    }

    // 状态获取
    std::vector<double> getCurrentJoints() override {
        return current_joints_;
    }
    
    geometry_msgs::msg::Pose getCurrentPose() override {
        return current_pose_;
    }
    
    elu_robot_arm_framework::RobotState getStatus() override {
        return connected_ ? elu_robot_arm_framework::RobotState::IDLE 
                          : elu_robot_arm_framework::RobotState::DISCONNECTED;
    }
    
    std::string getErrorMessage() override {
        return " ";
    }

    // 控制参数设置 (模拟实现)
    bool setSpeed(double speed_ratio) override {
        (void)speed_ratio;
        return true;
    }
    
    bool setAcceleration(double acceleration_ratio) override {
        (void)acceleration_ratio;
        return true;
    }
    
    bool setPayload(double payload_kg) override {
        (void)payload_kg;
        return true;
    }

    // 安全控制
    bool emergencyStop() override {
        return true;
    }
    
    bool clearError() override {
        return true;
    }
    
    bool enable() override {
        return true;
    }
    
    bool disable() override {
        return true;
    }

    // 信息获取
    std::string getRobotModel() const override {
        return "MockRobot";
    }
    
    int getDoF() const override {
        return 6;
    }
    
    double getMaxPayload() const override {
        return 5.0;  // 5kg
    }
    
    std::vector<double> getJointLimits() const override {
        return {-3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14};
    }

private:
    bool connected_;
    std::vector<double> current_joints_;
    geometry_msgs::msg::Pose current_pose_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<VisionCommunicationNode>();
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "🎯 Vision Communication Node Started");
        RCLCPP_INFO(rclcpp::get_logger("main"), "📡 Ready for TCP communication with vision system");
        
        // 设置信号处理器
        std::signal(SIGINT, [](int signal) {
            (void)signal;
            RCLCPP_INFO(rclcpp::get_logger("main"), "Received shutdown signal, gracefully shutting down...");
            rclcpp::shutdown();
        });
        
        // 运行节点
        rclcpp::spin(node);
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "✅ Vision Communication Node shutdown complete");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ Exception in main: %s", e.what());
        return 1;
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ Unknown exception in main");
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}