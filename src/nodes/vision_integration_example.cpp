/**
 * @file vision_integration_example.cpp
 * @brief 机器人视觉集成示例节点
 * 
 * 演示如何集成视觉通信到Rokae机械臂控制系统中
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "elu_robot_arm_framework/adapters/rokae_adapter.hpp"
#include "elu_robot_arm_framework/vision/vision_communication.hpp"
#include "elu_robot_arm_framework/msg/vision_result.hpp"
#include "elu_robot_arm_framework/srv/trigger_vision.hpp"

#include <thread>
#include <chrono>

class VisionIntegrationExample : public rclcpp::Node
{
public:
    VisionIntegrationExample() : Node("vision_integration_example")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Vision Integration Example");

        // 声明参数
        this->declare_parameter("config_file", "config/robots/rokae_cr7_config.yaml");
        this->declare_parameter("vision_server_ip", "192.168.1.100");
        this->declare_parameter("vision_server_port", 8080);
        this->declare_parameter("auto_start", false);
        this->declare_parameter("demo_mode", true);

        // 获取参数
        config_file_ = this->get_parameter("config_file").as_string();
        vision_server_ip_ = this->get_parameter("vision_server_ip").as_string();
        vision_server_port_ = this->get_parameter("vision_server_port").as_int();
        auto_start_ = this->get_parameter("auto_start").as_bool();
        demo_mode_ = this->get_parameter("demo_mode").as_bool();

        // 初始化机械臂适配器
        robot_adapter_ = std::make_shared<elu_robot_arm_framework::RokaeAdapter>();
        
        // 初始化视觉通信
        vision_comm_ = std::make_shared<elu_robot_arm_framework::VisionCommunication>(robot_adapter_);

        // 订阅视觉结果
        vision_result_sub_ = this->create_subscription<elu_robot_arm_framework::msg::VisionResult>(
            "vision_result", 10,
            std::bind(&VisionIntegrationExample::visionResultCallback, this, std::placeholders::_1));

        // 订阅视觉连接状态
        vision_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "vision_connection_status", 10,
            std::bind(&VisionIntegrationExample::visionStatusCallback, this, std::placeholders::_1));

        // 创建服务
        start_demo_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_vision_demo",
            std::bind(&VisionIntegrationExample::startDemoCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // 创建客户端
        trigger_vision_client_ = this->create_client<elu_robot_arm_framework::srv::TriggerVision>("trigger_vision");
        connect_vision_client_ = this->create_client<std_srvs::srv::Trigger>("connect_vision");

        RCLCPP_INFO(this->get_logger(), "Vision Integration Example Node initialized");
        RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Vision server: %s:%d", vision_server_ip_.c_str(), vision_server_port_);

        // 自动启动
        if (auto_start_) {
            startup_timer_ = this->create_wall_timer(
                std::chrono::seconds(3),
                std::bind(&VisionIntegrationExample::autoStart, this));
        }
    }

    ~VisionIntegrationExample()
    {
        if (robot_adapter_) {
            robot_adapter_->disconnect();
        }
    }

private:
    // ==================== 自动启动 ====================
    void autoStart()
    {
        startup_timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "=== Auto Start Sequence ===");
        
        // 1. 连接机械臂
        if (connectRobot()) {
            RCLCPP_INFO(this->get_logger(), "✓ Robot connected successfully");
            
            // 2. 连接视觉系统
            if (connectVisionSystem()) {
                RCLCPP_INFO(this->get_logger(), "✓ Vision system connected successfully");
                
                // 3. 启动演示
                if (demo_mode_) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    startVisionDemo();
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Failed to connect vision system");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to connect robot");
        }
    }

    bool connectRobot()
    {
        try {
            RCLCPP_INFO(this->get_logger(), "Connecting to Rokae robot...");
            
            if (!robot_adapter_->connect(config_file_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to Rokae robot");
                return false;
            }

            // 设置运动参数
            robot_adapter_->setSpeed(0.3);
            robot_adapter_->setAcceleration(0.3);

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during robot connection: %s", e.what());
            return false;
        }
    }

    bool connectVisionSystem()
    {
        try {
            RCLCPP_INFO(this->get_logger(), "Connecting to vision system...");
            
            // 等待服务可用
            if (!connect_vision_client_->wait_for_service(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Vision connect service not available");
                return false;
            }

            // 调用连接服务
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = connect_vision_client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Vision connection successful: %s", response->message.c_str());
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Vision connection failed: %s", response->message.c_str());
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to call vision connect service");
                return false;
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during vision connection: %s", e.what());
            return false;
        }
    }

    // ==================== 演示程序 ====================
    void startVisionDemo()
    {
        RCLCPP_INFO(this->get_logger(), "🎯 Starting Vision Integration Demo");
        
        demo_step_ = 0;
        demo_timer_ = this->create_wall_timer(
            std::chrono::seconds(8),
            std::bind(&VisionIntegrationExample::runDemoStep, this));
    }

    void runDemoStep()
    {
        switch (demo_step_) {
            case 0:
                demoMoveToScanPosition();
                break;
            case 1:
                demoTriggerVisionDetection();
                break;
            case 2:
                demoProcessVisionResults();
                break;
            case 3:
                demoPickAndPlace();
                break;
            case 4:
                demoReturnHome();
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "🎉 Vision Integration Demo completed!");
                demo_timer_->cancel();
                return;
        }
        demo_step_++;
    }

    void demoMoveToScanPosition()
    {
        RCLCPP_INFO(this->get_logger(), "=== Demo Step 1: Move to Scan Position ===");
        
        try {
            // 移动到扫描位置
            std::vector<double> scan_position = {0.0, -0.3, 0.5, 0.0, 1.57, 0.0};
            
            RCLCPP_INFO(this->get_logger(), "Moving to scan position...");
            
            if (robot_adapter_->moveToJoint(scan_position)) {
                RCLCPP_INFO(this->get_logger(), "✓ Reached scan position");
                
                // 短暂等待稳定
                std::this_thread::sleep_for(std::chrono::seconds(2));
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Failed to move to scan position");
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in move to scan position: %s", e.what());
        }
    }

    void demoTriggerVisionDetection()
    {
        RCLCPP_INFO(this->get_logger(), "=== Demo Step 2: Trigger Vision Detection ===");
        
        try {
            // 等待视觉服务可用
            if (!trigger_vision_client_->wait_for_service(std::chrono::seconds(3))) {
                RCLCPP_ERROR(this->get_logger(), "Trigger vision service not available");
                return;
            }

            // 触发视觉检测
            auto request = std::make_shared<elu_robot_arm_framework::srv::TriggerVision::Request>();
            request->station_id = 1;  // 工位号1
            request->timeout = 15.0;  // 15秒超时
            
            RCLCPP_INFO(this->get_logger(), "Triggering vision detection for station 1...");
            
            auto future = trigger_vision_client_->async_send_request(request);
            
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "✓ Vision detection triggered successfully");
                    RCLCPP_INFO(this->get_logger(), "Result code: %d", response->result_code);
                    RCLCPP_INFO(this->get_logger(), "Detected %zu objects", response->poses.size());
                    
                    // 保存检测结果
                    last_vision_result_.poses = response->poses;
                    last_vision_result_.result_code = response->result_code;
                    last_vision_result_.success = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Vision detection failed: %s", response->message.c_str());
                    last_vision_result_.success = false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Failed to call trigger vision service");
                last_vision_result_.success = false;
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in trigger vision detection: %s", e.what());
            last_vision_result_.success = false;
        }
    }

    void demoProcessVisionResults()
    {
        RCLCPP_INFO(this->get_logger(), "=== Demo Step 3: Process Vision Results ===");
        
        if (!last_vision_result_.success) {
            RCLCPP_WARN(this->get_logger(), "No valid vision results to process");
            return;
        }

        switch (last_vision_result_.result_code) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "📦 Empty box detected - no objects to pick");
                break;
                
            case -1:
                RCLCPP_WARN(this->get_logger(), "⚠️ Point cloud detected but no coordinates extracted");
                break;
                
            case -2:
                RCLCPP_ERROR(this->get_logger(), "❌ Invalid command sent to vision system");
                break;
                
            case -3:
                RCLCPP_ERROR(this->get_logger(), "❌ Vision detection mode not enabled");
                break;
                
            default:
                if (last_vision_result_.result_code > 0) {
                    RCLCPP_INFO(this->get_logger(), "🎯 Detected %d objects:", last_vision_result_.result_code);
                    
                    for (size_t i = 0; i < last_vision_result_.poses.size(); ++i) {
                        const auto& pose = last_vision_result_.poses[i];
                        RCLCPP_INFO(this->get_logger(), "  Object %zu: pos[%.3f, %.3f, %.3f] orient[%.3f, %.3f, %.3f, %.3f]",
                                    i + 1,
                                    pose.position.x, pose.position.y, pose.position.z,
                                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                    }
                    
                    // 选择第一个目标进行抓取
                    if (!last_vision_result_.poses.empty()) {
                        target_pose_ = last_vision_result_.poses[0];
                        RCLCPP_INFO(this->get_logger(), "✓ Selected first object as pick target");
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "⚠️ Unknown result code: %d", last_vision_result_.result_code);
                }
                break;
        }
    }

    void demoPickAndPlace()
    {
        RCLCPP_INFO(this->get_logger(), "=== Demo Step 4: Pick and Place ===");
        
        if (!last_vision_result_.success || last_vision_result_.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No valid target for pick and place");
            return;
        }

        try {
            // 计算预抓取位置 (目标位置上方50mm)
            geometry_msgs::msg::Pose pre_pick_pose = target_pose_;
            pre_pick_pose.position.z += 0.05;  // 上方50mm
            
            RCLCPP_INFO(this->get_logger(), "Moving to pre-pick position...");
            
            if (robot_adapter_->moveToPose(pre_pick_pose)) {
                RCLCPP_INFO(this->get_logger(), "✓ Reached pre-pick position");
                
                // 等待稳定
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                // 下降到抓取位置
                RCLCPP_INFO(this->get_logger(), "Moving to pick position...");
                
                if (robot_adapter_->linearMove(target_pose_)) {
                    RCLCPP_INFO(this->get_logger(), "✓ Reached pick position");
                    
                    // 模拟抓取动作 (实际应用中需要控制夹爪)
                    RCLCPP_INFO(this->get_logger(), "🤖 Simulating gripper close...");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    
                    // 提升到预抓取位置
                    RCLCPP_INFO(this->get_logger(), "Lifting object...");
                    
                    if (robot_adapter_->linearMove(pre_pick_pose)) {
                        RCLCPP_INFO(this->get_logger(), "✓ Object lifted successfully");
                        
                        // 移动到放置位置
                        geometry_msgs::msg::Pose place_pose = target_pose_;
                        place_pose.position.x += 0.2;  // 右移200mm
                        place_pose.position.z += 0.05; // 保持在上方
                        
                        RCLCPP_INFO(this->get_logger(), "Moving to place position...");
                        
                        if (robot_adapter_->moveToPose(place_pose)) {
                            RCLCPP_INFO(this->get_logger(), "✓ Reached place position");
                            
                            // 下降放置
                            place_pose.position.z = target_pose_.position.z;
                            
                            if (robot_adapter_->linearMove(place_pose)) {
                                RCLCPP_INFO(this->get_logger(), "✓ Object placed successfully");
                                
                                // 模拟夹爪打开
                                RCLCPP_INFO(this->get_logger(), "🤖 Simulating gripper open...");
                                std::this_thread::sleep_for(std::chrono::seconds(1));
                                
                                // 提升离开
                                place_pose.position.z += 0.05;
                                robot_adapter_->linearMove(place_pose);
                                
                                RCLCPP_INFO(this->get_logger(), "🎉 Pick and place completed successfully!");
                            }
                        }
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Failed to reach pick position");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Failed to reach pre-pick position");
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in pick and place: %s", e.what());
        }
    }

    void demoReturnHome()
    {
        RCLCPP_INFO(this->get_logger(), "=== Demo Step 5: Return Home ===");
        
        try {
            // 回到初始位置
            std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            
            RCLCPP_INFO(this->get_logger(), "Returning to home position...");
            
            robot_adapter_->setSpeed(0.2);  // 较慢速度返回
            
            if (robot_adapter_->moveToJoint(home_position)) {
                RCLCPP_INFO(this->get_logger(), "✓ Successfully returned home");
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Failed to return home");
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in return home: %s", e.what());
        }
    }

    // ==================== 回调函数 ====================
    void visionResultCallback(const elu_robot_arm_framework::msg::VisionResult::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "📸 Vision result received:");
        RCLCPP_INFO(this->get_logger(), "  Code: %d", msg->code);
        RCLCPP_INFO(this->get_logger(), "  Objects: %zu", msg->poses.size());
        
        if (!msg->error_message.empty()) {
            RCLCPP_INFO(this->get_logger(), "  Message: %s", msg->error_message.c_str());
        }
    }

    void visionStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        static std::string last_status;
        if (msg->data != last_status) {
            RCLCPP_INFO(this->get_logger(), "🔗 Vision connection status: %s", msg->data.c_str());
            last_status = msg->data;
        }
    }

    void startDemoCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (!robot_adapter_->isConnected()) {
            response->success = false;
            response->message = "Robot not connected";
            return;
        }

        if (!vision_comm_->isConnected()) {
            response->success = false;
            response->message = "Vision system not connected";
            return;
        }

        // 启动演示
        startVisionDemo();
        
        response->success = true;
        response->message = "Vision demo started";
    }

private:
    // 机器人相关
    std::shared_ptr<elu_robot_arm_framework::RokaeAdapter> robot_adapter_;
    std::shared_ptr<elu_robot_arm_framework::VisionCommunication> vision_comm_;
    
    // 配置参数
    std::string config_file_;
    std::string vision_server_ip_;
    int vision_server_port_;
    bool auto_start_;
    bool demo_mode_;
    
    // ROS2 接口
    rclcpp::Subscription<elu_robot_arm_framework::msg::VisionResult>::SharedPtr vision_result_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_status_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_demo_service_;
    
    rclcpp::Client<elu_robot_arm_framework::srv::TriggerVision>::SharedPtr trigger_vision_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_vision_client_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr demo_timer_;
    
    // 演示状态
    int demo_step_ = 0;
    
    // 视觉结果
    struct VisionResult {
        bool success = false;
        int result_code = 0;
        std::vector<geometry_msgs::msg::Pose> poses;
    } last_vision_result_;
    
    geometry_msgs::msg::Pose target_pose_;  // 目标抓取位姿
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<VisionIntegrationExample>();
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "🤖🎯 Starting Vision Integration Example Node");
        RCLCPP_INFO(rclcpp::get_logger("main"), "📋 Available services:");
        RCLCPP_INFO(rclcpp::get_logger("main"), "   - /start_vision_demo");
        RCLCPP_INFO(rclcpp::get_logger("main"), "   - /trigger_vision");
        RCLCPP_INFO(rclcpp::get_logger("main"), "   - /connect_vision");
        RCLCPP_INFO(rclcpp::get_logger("main"), "   - /disconnect_vision");
        RCLCPP_INFO(rclcpp::get_logger("main"), "📊 Subscribing to topics:");
        RCLCPP_INFO(rclcpp::get_logger("main"), "   - /vision_result");
        RCLCPP_INFO(rclcpp::get_logger("main"), "   - /vision_connection_status");
        
        // 设置信号处理器
        std::signal(SIGINT, [](int signal) {
            (void)signal;
            RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
            rclcpp::shutdown();
        });
        
        // 运行节点
        rclcpp::spin(node);
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "✅ Node shutdown complete");
        
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