/**
 * @file motion_controller.cpp
 * @brief 运动控制管理器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_robot_arm_framework/controllers/motion_controller.hpp"
#include <fstream>

namespace elu_robot_arm_framework
{

MotionController::MotionController(const rclcpp::NodeOptions& options)
: Node("motion_controller", options)
{
  RCLCPP_INFO(get_logger(), "Initializing Motion Controller...");

  // 初始化插件管理器
  plugin_manager_ = std::make_unique<PluginManager>();

  // 初始化安全检查器
  std::string safety_config = this->declare_parameter("safety_config", 
    std::string("config/safety/safety_config.yaml"));
  safety_checker_ = std::make_unique<SafetyChecker>(safety_config);

  // 创建发布者
  status_publisher_ = this->create_publisher<msg::RobotStatus>(
    "robot_status", 10);

  // 创建订阅者
  motion_subscriber_ = this->create_subscription<msg::MotionCommand>(
    "motion_command", 10,
    std::bind(&MotionController::motionCommandCallback, this, std::placeholders::_1));

  // 创建服务 - 使用正确的命名空间
  switch_robot_service_ = this->create_service<srv::SwitchRobot>(
    "switch_robot",
    std::bind(&MotionController::switchRobotCallback, this, 
              std::placeholders::_1, std::placeholders::_2));

  execute_motion_service_ = this->create_service<srv::ExecuteMotion>(
    "execute_motion",
    std::bind(&MotionController::executeMotionCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建状态发布定时器
  status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MotionController::statusTimerCallback, this));

  // 加载机械臂配置
  loadRobotConfigs();

  RCLCPP_INFO(get_logger(), "Motion Controller initialized successfully");
}

MotionController::~MotionController()
{
  // 断开所有连接的机械臂
  for (auto& [robot_id, robot] : loaded_robots_) {
    if (robot && robot->isConnected()) {
      robot->disconnect();
    }
  }
}

bool MotionController::loadRobot(const std::string& robot_id, const std::string& config_file)
{
  try {
    // 简化配置读取 - 避免yaml-cpp依赖问题
    RCLCPP_INFO(get_logger(), "Loading robot: %s from config: %s", 
                robot_id.c_str(), config_file.c_str());

    // 默认使用ELU适配器
    std::string plugin_name = "elu_adapter/EluAdapter";

    // 加载插件
    if (!plugin_manager_->loadPlugin(robot_id, plugin_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to load plugin for robot: %s", robot_id.c_str());
      return false;
    }

    // 获取插件实例
    auto robot = plugin_manager_->getPlugin(robot_id);
    if (!robot) {
      RCLCPP_ERROR(get_logger(), "Failed to get plugin instance for robot: %s", robot_id.c_str());
      return false;
    }

    // 连接机械臂
    if (!robot->connect(config_file)) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to robot: %s", robot_id.c_str());
      return false;
    }

    // 添加到已加载机械臂列表
    loaded_robots_[robot_id] = robot;

    RCLCPP_INFO(get_logger(), "Robot loaded successfully: %s", robot_id.c_str());
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception while loading robot %s: %s", 
                 robot_id.c_str(), e.what());
    return false;
  }
}

bool MotionController::switchRobot(const std::string& robot_id)
{
  if (loaded_robots_.find(robot_id) == loaded_robots_.end()) {
    RCLCPP_ERROR(get_logger(), "Robot not loaded: %s", robot_id.c_str());
    return false;
  }

  current_robot_ = loaded_robots_[robot_id];
  current_robot_id_ = robot_id;

  RCLCPP_INFO(get_logger(), "Switched to robot: %s", robot_id.c_str());
  return true;
}

bool MotionController::unloadRobot(const std::string& robot_id)
{
  auto it = loaded_robots_.find(robot_id);
  if (it == loaded_robots_.end()) {
    return false;
  }

  if (it->second && it->second->isConnected()) {
    it->second->disconnect();
  }

  loaded_robots_.erase(it);
  
  if (current_robot_id_ == robot_id) {
    current_robot_.reset();
    current_robot_id_.clear();
  }

  return plugin_manager_->unloadPlugin(robot_id);
}

std::string MotionController::getCurrentRobotId() const
{
  return current_robot_id_;
}

bool MotionController::executeMotion(const msg::MotionCommand& command)
{
  if (!current_robot_) {
    RCLCPP_ERROR(get_logger(), "No robot selected");
    return false;
  }

  // 安全检查
  auto safety_result = safety_checker_->checkMotionCommand(command);
  if (!safety_result.is_safe) {
    RCLCPP_ERROR(get_logger(), "Safety check failed: %s", 
                 safety_result.error_message.c_str());
    return false;
  }

  // 执行运动
  bool success = false;
  switch (command.motion_type) {
    case 0: // JOINT_MOVE
      success = current_robot_->moveToJoint(command.joint_positions, command.speed_ratio);
      break;
    case 1: // CARTESIAN_MOVE
      success = current_robot_->moveToPose(command.target_pose, command.speed_ratio);
      break;
    case 2: // LINEAR_MOVE
      success = current_robot_->linearMove(command.target_pose, command.speed_ratio);
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unsupported motion type: %d", command.motion_type);
      return false;
  }

  return success;
}

bool MotionController::emergencyStop()
{
  if (current_robot_) {
    return current_robot_->emergencyStop();
  }
  return false;
}

bool MotionController::clearError()
{
  if (current_robot_) {
    return current_robot_->clearError();
  }
  return false;
}

msg::RobotStatus MotionController::getRobotStatus(const std::string& robot_id)
{
  msg::RobotStatus status;
  
  std::string target_id = robot_id.empty() ? current_robot_id_ : robot_id;
  
  if (target_id.empty() || loaded_robots_.find(target_id) == loaded_robots_.end()) {
    status.robot_id = target_id;
    status.is_connected = false;
    status.status = static_cast<int32_t>(RobotState::DISCONNECTED);
    return status;
  }

  auto robot = loaded_robots_[target_id];
  status.robot_id = target_id;
  status.robot_model = robot->getRobotModel();
  status.is_connected = robot->isConnected();
  status.status = static_cast<int32_t>(robot->getStatus());
  status.current_joints = robot->getCurrentJoints();
  status.current_pose = robot->getCurrentPose();
  status.error_message = robot->getErrorMessage();

  return status;
}

std::vector<std::string> MotionController::getLoadedRobots() const
{
  std::vector<std::string> robots;
  for (const auto& [robot_id, robot] : loaded_robots_) {
    robots.push_back(robot_id);
  }
  return robots;
}

void MotionController::motionCommandCallback(const msg::MotionCommand::SharedPtr msg)
{
  if (!validateMotionCommand(*msg)) {
    RCLCPP_WARN(get_logger(), "Invalid motion command received");
    return;
  }

  executeMotion(*msg);
}

void MotionController::switchRobotCallback(
  const std::shared_ptr<srv::SwitchRobot::Request> request,
  std::shared_ptr<srv::SwitchRobot::Response> response)
{
  response->previous_robot = current_robot_id_;
  response->success = switchRobot(request->robot_id);
  
  if (response->success) {
    response->message = "Successfully switched to robot: " + request->robot_id;
  } else {
    response->message = "Failed to switch to robot: " + request->robot_id;
  }
}

void MotionController::executeMotionCallback(
  const std::shared_ptr<srv::ExecuteMotion::Request> request,
  std::shared_ptr<srv::ExecuteMotion::Response> response)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  // 创建运动命令
  msg::MotionCommand command;
  command.robot_id = request->robot_id;
  command.motion_type = request->motion_type;
  command.joint_positions = request->joint_positions;
  command.target_pose = request->target_pose;
  command.speed_ratio = request->speed_ratio;
  command.acceleration_ratio = request->acceleration_ratio;
  command.wait_for_completion = request->wait_for_completion;

  response->success = executeMotion(command);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  response->execution_time = duration.count() / 1000.0;

  if (response->success) {
    response->message = "Motion executed successfully";
  } else {
    response->message = "Motion execution failed";
  }
}

void MotionController::statusTimerCallback()
{
  if (!current_robot_) {
    return;
  }

  // 发布机械臂状态
  auto status_msg = getRobotStatus();
  status_msg.header.stamp = this->now();
  status_publisher_->publish(status_msg);
}

void MotionController::loadRobotConfigs()
{
  // 简化配置加载 - 使用默认配置
  RCLCPP_INFO(get_logger(), "Loading default robot configurations...");
  
  // 自动加载默认ELU机械臂
  std::string default_config = "config/robots/robot_config.yaml";
  loadRobot("elu_arm_1", default_config);
  switchRobot("elu_arm_1");
}

bool MotionController::validateMotionCommand(const msg::MotionCommand& command)
{
  // 基本验证
  if (command.speed_ratio < 0.0 || command.speed_ratio > 1.0) {
    RCLCPP_WARN(get_logger(), "Invalid speed ratio: %f", command.speed_ratio);
    return false;
  }

  if (command.acceleration_ratio < 0.0 || command.acceleration_ratio > 1.0) {
    RCLCPP_WARN(get_logger(), "Invalid acceleration ratio: %f", command.acceleration_ratio);
    return false;
  }

  return true;
}

} // namespace elu_robot_arm_framework
