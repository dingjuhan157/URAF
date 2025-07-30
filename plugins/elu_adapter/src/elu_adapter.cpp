/**
 * @file elu_adapter.cpp
 * @brief ELU机械臂适配器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_adapter/elu_adapter.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace elu_adapter
{

EluAdapter::EluAdapter()
: connected_(false)
, current_state_(elu_robot_arm_framework::RobotState::DISCONNECTED)
, current_speed_ratio_(1.0)
, current_acceleration_ratio_(1.0)
, current_payload_(0.0)
, logger_(rclcpp::get_logger("elu_adapter"))
{
  // 初始化默认关节角度（6自由度）
  current_joints_.resize(6, 0.0);

  // 初始化默认位姿
  current_pose_.position.x = 0.5;
  current_pose_.position.y = 0.0;
  current_pose_.position.z = 0.5;
  current_pose_.orientation.w = 1.0;
  current_pose_.orientation.x = 0.0;
  current_pose_.orientation.y = 0.0;
  current_pose_.orientation.z = 0.0;

  RCLCPP_INFO(logger_, "ELU Adapter initialized");
}

EluAdapter::~EluAdapter()
{
  if (connected_) {
    disconnect();
  }
}

bool EluAdapter::connect(const std::string& config_file)
{
  try {
    // 简化配置读取 - 使用默认参数
    ip_address_ = "192.168.1.100";
    port_ = 8080;

    RCLCPP_INFO(logger_, "Connecting to ELU robot at %s:%d", ip_address_.c_str(), port_);

    // TODO: 实际连接ELU机械臂
    // 这里应该调用ELU SDK的连接函数
    
    // 模拟连接过程
    current_state_ = elu_robot_arm_framework::RobotState::CONNECTING;
    
    // 初始化连接
    if (!initializeConnection()) {
      handleError("Failed to initialize connection");
      return false;
    }

    connected_ = true;
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
    error_message_.clear();

    RCLCPP_INFO(logger_, "Successfully connected to ELU robot");
    return true;
  }
  catch (const std::exception& e) {
    handleError("Connection failed: " + std::string(e.what()));
    return false;
  }
}

bool EluAdapter::disconnect()
{
  if (!connected_) {
    return true;
  }

  try {
    // TODO: 实际断开ELU机械臂连接
    // 这里应该调用ELU SDK的断开函数

    connected_ = false;
    current_state_ = elu_robot_arm_framework::RobotState::DISCONNECTED;

    RCLCPP_INFO(logger_, "Disconnected from ELU robot");
    return true;
  }
  catch (const std::exception& e) {
    handleError("Disconnection failed: " + std::string(e.what()));
    return false;
  }
}

bool EluAdapter::isConnected() const
{
  return connected_;
}

bool EluAdapter::moveToJoint(const std::vector<double>& joints, double speed_ratio)
{
  if (!connected_) {
    handleError("Robot not connected");
    return false;
  }

  if (joints.size() != 6) {
    handleError("Invalid joint array size: expected 6, got " + std::to_string(joints.size()));
    return false;
  }

  try {
    RCLCPP_INFO(logger_, "Moving to joint positions with speed ratio: %f", speed_ratio);

    current_state_ = elu_robot_arm_framework::RobotState::MOVING;

    // TODO: 实际执行关节运动
    // 这里应该调用ELU SDK的关节运动函数

    // 模拟运动过程
    current_joints_ = joints;
    
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
    return true;
  }
  catch (const std::exception& e) {
    handleError("Joint movement failed: " + std::string(e.what()));
    return false;
  }
}

bool EluAdapter::moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio)
{
  if (!connected_) {
    handleError("Robot not connected");
    return false;
  }

  try {
    RCLCPP_INFO(logger_, "Moving to pose [%f, %f, %f] with speed ratio: %f",
                pose.position.x, pose.position.y, pose.position.z, speed_ratio);

    current_state_ = elu_robot_arm_framework::RobotState::MOVING;

    // TODO: 实际执行笛卡尔运动
    current_pose_ = pose;
    
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
    return true;
  }
  catch (const std::exception& e) {
    handleError("Pose movement failed: " + std::string(e.what()));
    return false;
  }
}

bool EluAdapter::linearMove(const geometry_msgs::msg::Pose& pose, double speed_ratio)
{
  if (!connected_) {
    handleError("Robot not connected");
    return false;
  }

  try {
    RCLCPP_INFO(logger_, "Linear move to pose [%f, %f, %f] with speed ratio: %f",
                pose.position.x, pose.position.y, pose.position.z, speed_ratio);

    current_state_ = elu_robot_arm_framework::RobotState::MOVING;

    // TODO: 实际执行直线运动
    current_pose_ = pose;
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
    return true;
  }
  catch (const std::exception& e) {
    handleError("Linear movement failed: " + std::string(e.what()));
    return false;
  }
}

std::vector<double> EluAdapter::getCurrentJoints()
{
  if (!connected_) {
    return {};
  }

  updateRobotState();
  return current_joints_;
}

geometry_msgs::msg::Pose EluAdapter::getCurrentPose()
{
  if (!connected_) {
    return geometry_msgs::msg::Pose();
  }

  updateRobotState();
  return current_pose_;
}

elu_robot_arm_framework::RobotState EluAdapter::getStatus()
{
  if (connected_) {
    updateRobotState();
  }
  return current_state_;
}

std::string EluAdapter::getErrorMessage()
{
  return error_message_;
}

bool EluAdapter::setSpeed(double speed_ratio)
{
  if (speed_ratio < 0.0 || speed_ratio > 1.0) {
    handleError("Invalid speed ratio: " + std::to_string(speed_ratio));
    return false;
  }

  current_speed_ratio_ = speed_ratio;
  RCLCPP_INFO(logger_, "Speed ratio set to: %f", speed_ratio);
  return true;
}

bool EluAdapter::setAcceleration(double acceleration_ratio)
{
  if (acceleration_ratio < 0.0 || acceleration_ratio > 1.0) {
    handleError("Invalid acceleration ratio: " + std::to_string(acceleration_ratio));
    return false;
  }

  current_acceleration_ratio_ = acceleration_ratio;
  RCLCPP_INFO(logger_, "Acceleration ratio set to: %f", acceleration_ratio);
  return true;
}

bool EluAdapter::setPayload(double payload_kg)
{
  if (payload_kg < 0.0 || payload_kg > getMaxPayload()) {
    handleError("Invalid payload: " + std::to_string(payload_kg));
    return false;
  }

  current_payload_ = payload_kg;
  RCLCPP_INFO(logger_, "Payload set to: %f kg", payload_kg);
  return true;
}

bool EluAdapter::emergencyStop()
{
  RCLCPP_WARN(logger_, "Emergency stop triggered!");

  // TODO: 调用ELU SDK的紧急停止函数

  current_state_ = elu_robot_arm_framework::RobotState::EMERGENCY_STOP;
  return true;
}

bool EluAdapter::clearError()
{
  error_message_.clear();
  if (current_state_ == elu_robot_arm_framework::RobotState::ERROR) {
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
  }
  RCLCPP_INFO(logger_, "Error cleared");
  return true;
}

bool EluAdapter::enable()
{
  if (!connected_) {
    handleError("Robot not connected");
    return false;
  }

  // TODO: 调用ELU SDK的使能函数

  if (current_state_ == elu_robot_arm_framework::RobotState::EMERGENCY_STOP) {
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
  }

  RCLCPP_INFO(logger_, "Robot enabled");
  return true;
}

bool EluAdapter::disable()
{
  if (!connected_) {
    return true;
  }

  // TODO: 调用ELU SDK的失能函数

  RCLCPP_INFO(logger_, "Robot disabled");
  return true;
}

std::string EluAdapter::getRobotModel() const
{
  return "ELU-6DOF";
}

int EluAdapter::getDoF() const
{
  return 6;
}

double EluAdapter::getMaxPayload() const
{
  return 5.0; // 5kg
}

std::vector<double> EluAdapter::getJointLimits() const
{
  // 返回关节限制 [min1, max1, min2, max2, ...]
  return {-3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14};
}

bool EluAdapter::initializeConnection()
{
  // TODO: 实现ELU机械臂的初始化逻辑
  RCLCPP_INFO(logger_, "Initializing connection to ELU robot...");
  return true;
}

bool EluAdapter::updateRobotState()
{
  // TODO: 从ELU机械臂获取实时状态
  return true;
}

void EluAdapter::handleError(const std::string& error_msg)
{
  error_message_ = error_msg;
  current_state_ = elu_robot_arm_framework::RobotState::ERROR;
  RCLCPP_ERROR(logger_, "%s", error_msg.c_str());
}

} // namespace elu_adapter

// 注册插件
PLUGINLIB_EXPORT_CLASS(elu_adapter::EluAdapter, elu_robot_arm_framework::RobotArmInterface)
