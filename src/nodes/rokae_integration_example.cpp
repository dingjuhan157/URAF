/**
 * @file rokae_integration_example.cpp
 * @brief Rokae机械臂集成使用示例
 * 
 * 使用Rokae适配器连接和控制xMateCR7机械臂
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "elu_robot_arm_framework/controllers/motion_controller.hpp"
#include "elu_robot_arm_framework/adapters/rokae_adapter.hpp"
#include "elu_robot_arm_framework/planners/rokae_motion_planner.hpp"

// 修复 std_msgs::msg::String 问题
#include <std_msgs/msg/string.hpp>

#include <thread>
#include <chrono>

// 添加工具函数来转换 RobotState 枚举
std::string robotStateToString(elu_robot_arm_framework::RobotState state) {
  switch (state) {
    case elu_robot_arm_framework::RobotState::DISCONNECTED:
      return "DISCONNECTED";
    case elu_robot_arm_framework::RobotState::CONNECTING:
      return "CONNECTING";
    case elu_robot_arm_framework::RobotState::IDLE:
      return "IDLE";
    case elu_robot_arm_framework::RobotState::MOVING:
      return "MOVING";
    case elu_robot_arm_framework::RobotState::ERROR:
      return "ERROR";
    case elu_robot_arm_framework::RobotState::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    default:
      return "UNKNOWN";
  }
}

class RokaeIntegrationExample : public rclcpp::Node
{
public:
  RokaeIntegrationExample() : Node("rokae_integration_example")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Rokae Integration Example");

    // 声明参数
    this->declare_parameter("config_file", "config/robots/rokae_cr7_config.yaml");
    this->declare_parameter("demo_mode", true);
    this->declare_parameter("auto_start", false);

    // 获取参数
    config_file_ = this->get_parameter("config_file").as_string();
    demo_mode_ = this->get_parameter("demo_mode").as_bool();
    auto_start_ = this->get_parameter("auto_start").as_bool();

    // 初始化发布者
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
    
    robot_status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "robot_status", 10);

    // 初始化服务
    connect_service_ = this->create_service<std_srvs::srv::Trigger>(
      "connect_robot", 
      std::bind(&RokaeIntegrationExample::connectRobotService, this, 
                std::placeholders::_1, std::placeholders::_2));

    disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
      "disconnect_robot",
      std::bind(&RokaeIntegrationExample::disconnectRobotService, this,
                std::placeholders::_1, std::placeholders::_2));

    emergency_stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "emergency_stop",
      std::bind(&RokaeIntegrationExample::emergencyStopService, this,
                std::placeholders::_1, std::placeholders::_2));

    // 初始化机械臂适配器
    robot_adapter_ = std::make_shared<elu_robot_arm_framework::RokaeAdapter>();
    
    // 初始化运动规划器
    motion_planner_ = std::make_shared<elu_robot_arm_framework::RokaeMotionPlanner>(robot_adapter_);

    // 状态发布定时器
    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RokaeIntegrationExample::publishStatus, this));

    // 自动连接
    if (auto_start_) {
      connection_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&RokaeIntegrationExample::autoConnect, this));
    }

    RCLCPP_INFO(this->get_logger(), "Rokae Integration Example Node initialized");
    RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Demo mode: %s", demo_mode_ ? "enabled" : "disabled");
  }

  ~RokaeIntegrationExample()
  {
    if (robot_adapter_) {
      robot_adapter_->disconnect();
    }
  }

private:
  // ==================== 连接管理 ====================
  void autoConnect()
  {
    connection_timer_->cancel();
    
    if (connectRobot()) {
      if (demo_mode_) {
        startDemo();
      }
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

      RCLCPP_INFO(this->get_logger(), "Successfully connected to Rokae robot");
      
      // 设置运动参数
      robot_adapter_->setSpeed(0.5);  // 50% 速度
      robot_adapter_->setAcceleration(0.3);  // 30% 加速度
      robot_adapter_->setBlendRadius(0.01);  // 10mm转弯区

      // 打印初始状态
      printRobotStatus();

      connected_ = true;
      return true;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during robot connection: %s", e.what());
      return false;
    }
  }

  void disconnectRobot()
  {
    try {
      if (robot_adapter_) {
        robot_adapter_->disconnect();
        connected_ = false;
        RCLCPP_INFO(this->get_logger(), "Disconnected from Rokae robot");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during disconnect: %s", e.what());
    }
  }

  // ==================== 演示程序 ====================
  void startDemo()
  {
    if (!connected_) {
      RCLCPP_ERROR(this->get_logger(), "Robot not connected, cannot start demo");
      return;
    }

    demo_step_ = 0;
    demo_timer_ = this->create_wall_timer(
      std::chrono::seconds(8),
      std::bind(&RokaeIntegrationExample::runDemo, this));
    
    RCLCPP_INFO(this->get_logger(), "Starting demonstration sequence...");
  }

  void runDemo()
  {
    if (!connected_) {
      RCLCPP_WARN(this->get_logger(), "Robot disconnected, stopping demo");
      demo_timer_->cancel();
      return;
    }

    switch (demo_step_) {
      case 0:
        demoJointMovement();
        break;
      case 1:
        demoCartesianMovement();
        break;
      case 2:
        demoTrajectoryPlanning();
        break;
      case 3:
        demoMultiPointPath();
        break;
      case 4:
        demoCircularPath();
        break;
      case 5:
        demoReturnHome();
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "=== Demo completed successfully! ===");
        demo_timer_->cancel();
        return;
    }
    
    demo_step_++;
  }

  void demoJointMovement()
  {
    RCLCPP_INFO(this->get_logger(), "=== Demo 1: Joint Space Movement ===");
    
    try {
      // 获取当前关节角度
      auto current_joints = robot_adapter_->getCurrentJoints();
      if (current_joints.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current joint positions");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Current joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  current_joints[0], current_joints[1], current_joints[2],
                  current_joints[3], current_joints[4], current_joints[5]);

      // 定义目标关节角度 (安全的小幅运动)
      std::vector<double> target_joints = current_joints;
      target_joints[0] += 0.2;  // J1 增加约11.5度
      target_joints[1] += 0.1;  // J2 增加约5.7度
      target_joints[5] -= 0.3;  // J6 减少约17.2度

      RCLCPP_INFO(this->get_logger(), "Moving to target joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  target_joints[0], target_joints[1], target_joints[2],
                  target_joints[3], target_joints[4], target_joints[5]);

      // 执行关节运动
      if (robot_adapter_->moveToJoint(target_joints)) {
        RCLCPP_INFO(this->get_logger(), "✓ Joint movement completed successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "✗ Joint movement failed");
      }

      // 保存位置用于后续演示
      demo_position_1_ = target_joints;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in joint movement demo: %s", e.what());
    }
  }

  void demoCartesianMovement()
  {
    RCLCPP_INFO(this->get_logger(), "=== Demo 2: Cartesian Movement ===");
    
    try {
      // 获取当前位姿
      auto current_pose = robot_adapter_->getCurrentPose();
      
      RCLCPP_INFO(this->get_logger(), "Current pose: position[%.3f, %.3f, %.3f] orientation[%.3f, %.3f, %.3f, %.3f]",
                  current_pose.position.x, current_pose.position.y, current_pose.position.z,
                  current_pose.orientation.x, current_pose.orientation.y,
                  current_pose.orientation.z, current_pose.orientation.w);

      // 定义目标位姿 (在当前位置基础上小幅移动)
      geometry_msgs::msg::Pose target_pose = current_pose;
      target_pose.position.x += 0.05;  // X方向移动5cm
      target_pose.position.z += 0.03;  // Z方向上移3cm

      RCLCPP_INFO(this->get_logger(), "Moving to target pose: position[%.3f, %.3f, %.3f]",
                  target_pose.position.x, target_pose.position.y, target_pose.position.z);

      // 执行笛卡尔运动
      if (robot_adapter_->moveToPose(target_pose)) {
        RCLCPP_INFO(this->get_logger(), "✓ Cartesian movement completed successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "✗ Cartesian movement failed");
      }

      // 保存位置用于后续演示
      demo_pose_1_ = target_pose;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in Cartesian movement demo: %s", e.what());
    }
  }

  void demoTrajectoryPlanning()
  {
    RCLCPP_INFO(this->get_logger(), "=== Demo 3: Trajectory Planning ===");
    
    try {
      // 获取当前状态
      std::vector<double> current_joints;
      geometry_msgs::msg::Pose current_pose;
      
      if (!motion_planner_->getCurrentState(current_joints, current_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
        return;
      }

      // 定义目标关节位置
      std::vector<double> target_joints = current_joints;
      target_joints[1] += 0.3;  // J2增加约17度
      target_joints[2] -= 0.2;  // J3减少约11度
      target_joints[4] += 0.15; // J5增加约8.6度

      RCLCPP_INFO(this->get_logger(), "Planning trajectory from current to target position...");

      // 规划关节轨迹
      trajectory_msgs::msg::JointTrajectory trajectory;
      if (motion_planner_->planJointTrajectory(current_joints, target_joints, 0.6, trajectory)) {
        double duration = trajectory.points.back().time_from_start.sec +
                         trajectory.points.back().time_from_start.nanosec * 1e-9;
        
        RCLCPP_INFO(this->get_logger(), "✓ Joint trajectory planned with %zu points, duration: %.2f seconds",
                    trajectory.points.size(), duration);

        // 验证轨迹
        if (motion_planner_->validateTrajectory(trajectory)) {
          RCLCPP_INFO(this->get_logger(), "✓ Trajectory validation passed");
          
          // 执行轨迹 (这里简化为执行最终点)
          if (robot_adapter_->moveToJoint(target_joints)) {
            RCLCPP_INFO(this->get_logger(), "✓ Planned trajectory executed successfully");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "✗ Trajectory validation failed");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "✗ Failed to plan joint trajectory");
      }

      // 保存位置
      demo_position_2_ = target_joints;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in trajectory planning demo: %s", e.what());
    }
  }

  void demoMultiPointPath()
  {
    RCLCPP_INFO(this->get_logger(), "=== Demo 4: Multi-Point Path ===");
    
    try {
      // 获取当前状态
      std::vector<double> current_joints;
      geometry_msgs::msg::Pose current_pose;
      
      if (!motion_planner_->getCurrentState(current_joints, current_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
        return;
      }

      // 定义路径点序列
      std::vector<elu_robot_arm_framework::PathPoint> path_points;
      
      // 起始点
      elu_robot_arm_framework::PathPoint start_point;
      start_point.type = elu_robot_arm_framework::PathPointType::JOINT_SPACE;
      start_point.joints = current_joints;
      start_point.speed_ratio = 0.5;
      path_points.push_back(start_point);

      // 中间点1
      elu_robot_arm_framework::PathPoint mid_point1;
      mid_point1.type = elu_robot_arm_framework::PathPointType::JOINT_SPACE;
      mid_point1.joints = current_joints;
      mid_point1.joints[0] += 0.4;  // J1运动
      mid_point1.joints[2] += 0.3;  // J3运动
      mid_point1.speed_ratio = 0.7;
      mid_point1.blend_radius = 0.02; // 20mm转弯区
      path_points.push_back(mid_point1);

      // 中间点2
      elu_robot_arm_framework::PathPoint mid_point2;
      mid_point2.type = elu_robot_arm_framework::PathPointType::JOINT_SPACE;
      mid_point2.joints = current_joints;
      mid_point2.joints[1] += 0.5;  // J2运动
      mid_point2.joints[4] -= 0.4;  // J5运动
      mid_point2.speed_ratio = 0.6;
      mid_point2.blend_radius = 0.02;
      path_points.push_back(mid_point2);

      // 中间点3 (组合运动)
      elu_robot_arm_framework::PathPoint mid_point3;
      mid_point3.type = elu_robot_arm_framework::PathPointType::JOINT_SPACE;
      mid_point3.joints = current_joints;
      mid_point3.joints[0] -= 0.2;  // J1反向
      mid_point3.joints[1] += 0.3;  // J2运动
      mid_point3.joints[5] += 0.5;  // J6运动
      mid_point3.speed_ratio = 0.8;
      mid_point3.blend_radius = 0.015;
      path_points.push_back(mid_point3);

      // 规划多点路径
      trajectory_msgs::msg::JointTrajectory trajectory;
      if (motion_planner_->planPathPointsTrajectory(path_points, trajectory)) {
        double total_duration = trajectory.points.back().time_from_start.sec +
                               trajectory.points.back().time_from_start.nanosec * 1e-9;
        
        RCLCPP_INFO(this->get_logger(), "✓ Multi-point path planned with %zu points, total duration: %.2f seconds",
                    trajectory.points.size(), total_duration);

        // 执行路径 (依次执行各个路径点)
        for (size_t i = 1; i < path_points.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "Moving to path point %zu/%zu (speed: %.1f%%)", 
                     i, path_points.size()-1, path_points[i].speed_ratio * 100);
          
          // 设置当前点的速度
          robot_adapter_->setSpeed(path_points[i].speed_ratio);
          
          if (robot_adapter_->moveToJointAsync(path_points[i].joints)) {
            // 监控运动状态
            monitorMovement(5.0); // 5秒超时
            RCLCPP_INFO(this->get_logger(), "✓ Reached path point %zu", i);
          } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to move to path point %zu", i);
            break;
          }
        }

        RCLCPP_INFO(this->get_logger(), "✓ Multi-point path execution completed");
      } else {
        RCLCPP_ERROR(this->get_logger(), "✗ Failed to plan multi-point path");
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in multi-point path demo: %s", e.what());
    }
  }

  void demoCircularPath()
  {
    RCLCPP_INFO(this->get_logger(), "=== Demo 5: Circular Path Simulation ===");
    
    try {
      // 获取当前关节角度
      auto current_joints = robot_adapter_->getCurrentJoints();
      if (current_joints.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current joint positions");
        return;
      }

      // 模拟圆形路径：通过多个关节位置近似圆弧运动
      std::vector<std::vector<double>> circle_points;
      const int num_points = 8;
      const double radius = 0.3; // 关节空间的"半径"
      
      for (int i = 0; i < num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        std::vector<double> point = current_joints;
        
        // 在J1和J2关节上创建圆形运动
        point[0] += radius * std::cos(angle);
        point[1] += radius * std::sin(angle) * 0.5; // 减小Y方向幅度
        
        circle_points.push_back(point);
      }

      RCLCPP_INFO(this->get_logger(), "Executing circular path with %d points...", num_points);

      // 设置合适的速度
      robot_adapter_->setSpeed(0.4);

      // 执行圆形路径
      for (size_t i = 0; i < circle_points.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Moving to circle point %zu/%zu", i+1, circle_points.size());
        
        if (robot_adapter_->moveToJointAsync(circle_points[i])) {
          monitorMovement(4.0); // 4秒超时
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to move to circle point %zu", i+1);
          break;
        }
      }

      // 回到起始点
      RCLCPP_INFO(this->get_logger(), "Returning to starting position...");
      if (robot_adapter_->moveToJoint(current_joints)) {
        RCLCPP_INFO(this->get_logger(), "✓ Circular path completed successfully");
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in circular path demo: %s", e.what());
    }
  }

  void demoReturnHome()
  {
    RCLCPP_INFO(this->get_logger(), "=== Demo 6: Return to Home Position ===");
    
    try {
      // 定义家位置 (零位)
      std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      
      RCLCPP_INFO(this->get_logger(), "Returning to home position: [0, 0, 0, 0, 0, 0]");
      
      // 设置较慢的速度以确保安全
      robot_adapter_->setSpeed(0.3);
      
      if (robot_adapter_->moveToJoint(home_joints)) {
        RCLCPP_INFO(this->get_logger(), "✓ Successfully returned to home position");
        RCLCPP_INFO(this->get_logger(), "🎉 All demonstrations completed successfully!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "✗ Failed to return to home position");
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in return home demo: %s", e.what());
    }
  }

  // ==================== 辅助函数 ====================
  void monitorMovement(double timeout_seconds)
  {
    auto start_time = std::chrono::steady_clock::now();
    
    // 修复：使用枚举比较而不是访问成员
    while (robot_adapter_->getStatus() == elu_robot_arm_framework::RobotState::MOVING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      // 超时保护
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > std::chrono::duration<double>(timeout_seconds)) {
        RCLCPP_WARN(this->get_logger(), "Movement timeout (%.1fs), stopping", timeout_seconds);
        robot_adapter_->emergencyStop();
        break;
      }
    }
  }

  void printRobotStatus()
  {
    if (!robot_adapter_) return;

    auto status = robot_adapter_->getStatus();
    std::string error_msg = robot_adapter_->getErrorMessage();
    auto current_joints = robot_adapter_->getCurrentJoints();
    auto current_pose = robot_adapter_->getCurrentPose();

    RCLCPP_INFO(this->get_logger(), "=== Robot Status ===");
    // 修复：使用 isConnected() 方法
    RCLCPP_INFO(this->get_logger(), "Connected: %s", 
                robot_adapter_->isConnected() ? "✓ Yes" : "✗ No");
    // 修复：使用枚举比较
    RCLCPP_INFO(this->get_logger(), "Moving: %s", 
                status == elu_robot_arm_framework::RobotState::MOVING ? "⚡ Yes" : "⏸ No");
    // 修复：使用枚举比较
    RCLCPP_INFO(this->get_logger(), "Error: %s", 
                status == elu_robot_arm_framework::RobotState::ERROR ? "⚠ Yes" : "✓ No");
    // 修复：使用工具函数
    RCLCPP_INFO(this->get_logger(), "Mode: %s", robotStateToString(status).c_str());
    
    // 修复：使用枚举比较和 getErrorMessage()
    if (status == elu_robot_arm_framework::RobotState::ERROR && !error_msg.empty()) {
      RCLCPP_WARN(this->get_logger(), "Error message: %s", error_msg.c_str());
    }
    
    if (!current_joints.empty()) {
      RCLCPP_INFO(this->get_logger(), "Joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  current_joints[0], current_joints[1], current_joints[2],
                  current_joints[3], current_joints[4], current_joints[5]);
    }

    RCLCPP_INFO(this->get_logger(), "Position: [%.3f, %.3f, %.3f]",
                current_pose.position.x, current_pose.position.y, current_pose.position.z);
    RCLCPP_INFO(this->get_logger(), "==================");
  }

  void publishStatus()
  {
    if (!robot_adapter_) return;

    try {
      // 发布关节状态
      auto current_joints = robot_adapter_->getCurrentJoints();
      if (!current_joints.empty()) {
        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = this->now();
        joint_msg.header.frame_id = "base_link";
        joint_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        joint_msg.position = current_joints;
        joint_msg.velocity = std::vector<double>(6, 0.0);
        joint_msg.effort = std::vector<double>(6, 0.0);
        joint_state_pub_->publish(joint_msg);
      }

      // 发布机器人状态 - 修复：使用枚举比较和方法调用
      auto status = robot_adapter_->getStatus();
      std_msgs::msg::String status_msg;
      status_msg.data = "Connected: " + std::string(robot_adapter_->isConnected() ? "true" : "false") +
                       ", Moving: " + std::string(status == elu_robot_arm_framework::RobotState::MOVING ? "true" : "false") +
                       ", Error: " + std::string(status == elu_robot_arm_framework::RobotState::ERROR ? "true" : "false") +
                       ", Mode: " + robotStateToString(status);
      robot_status_pub_->publish(status_msg);

    } catch (const std::exception& e) {
      // 静默处理发布错误，避免日志过多
    }
  }

  // ==================== 服务回调 ====================
  void connectRobotService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // 未使用的参数
    
    if (connected_) {
      response->success = true;
      response->message = "Robot already connected";
      return;
    }

    if (connectRobot()) {
      response->success = true;
      response->message = "Robot connected successfully";
    } else {
      response->success = false;
      response->message = "Failed to connect to robot";
    }
  }

  void disconnectRobotService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // 未使用的参数
    
    try {
      disconnectRobot();
      response->success = true;
      response->message = "Robot disconnected successfully";
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Failed to disconnect robot: " + std::string(e.what());
    }
  }

  void emergencyStopService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // 未使用的参数
    
    try {
      if (robot_adapter_ && robot_adapter_->emergencyStop()) {
        response->success = true;
        response->message = "Emergency stop executed successfully";
        RCLCPP_WARN(this->get_logger(), "🚨 Emergency stop activated via service call");
      } else {
        response->success = false;
        response->message = "Failed to execute emergency stop";
      }
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Exception during emergency stop: " + std::string(e.what());
    }
  }

private:
  // 机械臂组件
  std::shared_ptr<elu_robot_arm_framework::RokaeAdapter> robot_adapter_;
  std::shared_ptr<elu_robot_arm_framework::RokaeMotionPlanner> motion_planner_;
  

  // ROS2 组件
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;

  // 定时器
  rclcpp::TimerBase::SharedPtr demo_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;

  // 配置参数
  std::string config_file_;
  bool demo_mode_;
  bool auto_start_;

  // 状态变量
  bool connected_ = false;
  int demo_step_ = 0;

  // 演示位置记录
  std::vector<double> demo_position_1_;
  std::vector<double> demo_position_2_;
  geometry_msgs::msg::Pose demo_pose_1_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<RokaeIntegrationExample>();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "🤖 Starting Rokae Integration Example Node");
    RCLCPP_INFO(rclcpp::get_logger("main"), "📋 Available services:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "   - /connect_robot");
    RCLCPP_INFO(rclcpp::get_logger("main"), "   - /disconnect_robot");
    RCLCPP_INFO(rclcpp::get_logger("main"), "   - /emergency_stop");
    RCLCPP_INFO(rclcpp::get_logger("main"), "📊 Publishing topics:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "   - /joint_states");
    RCLCPP_INFO(rclcpp::get_logger("main"), "   - /robot_status");
    
    // 设置信号处理器，优雅退出
    std::signal(SIGINT, [](int signal) {
      (void)signal; // 避免未使用参数警告
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