/**
 * @file safety_checker.cpp
 * @brief 安全检查器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_robot_arm_framework/safety/safety_checker.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace elu_robot_arm_framework
{

SafetyChecker::SafetyChecker(const std::string& config_file)
: max_speed_ratio_(1.0)
, max_acceleration_ratio_(1.0)
, safety_enabled_(true)
, logger_(rclcpp::get_logger("safety_checker"))
{
  // 设置默认工作空间限制
  workspace_limits_.x_min = -1.0;
  workspace_limits_.x_max = 1.0;
  workspace_limits_.y_min = -1.0;
  workspace_limits_.y_max = 1.0;
  workspace_limits_.z_min = 0.0;
  workspace_limits_.z_max = 2.0;

  if (!config_file.empty()) {
    loadSafetyConfig(config_file);
  }

  RCLCPP_INFO(logger_, "Safety checker initialized");
}

SafetyCheckResult SafetyChecker::checkJointLimits(const std::vector<double>& joints, 
                                                 const std::vector<double>& joint_limits)
{
  SafetyCheckResult result;
  result.is_safe = true;
  result.error_code = 0;

  if (!safety_enabled_) {
    return result;
  }

  if (joints.size() != joint_limits.size() / 2) {
    result.is_safe = false;
    result.error_message = "Joint positions and limits size mismatch";
    result.error_code = -1;
    return result;
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    double min_limit = joint_limits[i * 2];
    double max_limit = joint_limits[i * 2 + 1];

    if (joints[i] < min_limit || joints[i] > max_limit) {
      result.is_safe = false;
      result.error_message = "Joint " + std::to_string(i) + " out of limits: " + 
                            std::to_string(joints[i]) + " not in [" + 
                            std::to_string(min_limit) + ", " + std::to_string(max_limit) + "]";
      result.error_code = i + 1;
      return result;
    }
  }

  return result;
}

SafetyCheckResult SafetyChecker::checkWorkspace(const geometry_msgs::msg::Pose& pose)
{
  SafetyCheckResult result;
  result.is_safe = true;
  result.error_code = 0;

  if (!safety_enabled_) {
    return result;
  }

  if (!isInWorkspace(pose)) {
    result.is_safe = false;
    result.error_message = "Target pose outside workspace limits";
    result.error_code = -2;
  }

  return result;
}

SafetyCheckResult SafetyChecker::checkSpeed(double speed_ratio)
{
  SafetyCheckResult result;
  result.is_safe = true;
  result.error_code = 0;

  if (!safety_enabled_) {
    return result;
  }

  if (speed_ratio < 0.0 || speed_ratio > max_speed_ratio_) {
    result.is_safe = false;
    result.error_message = "Speed ratio out of range: " + std::to_string(speed_ratio) + 
                          " not in [0.0, " + std::to_string(max_speed_ratio_) + "]";
    result.error_code = -3;
  }

  return result;
}

SafetyCheckResult SafetyChecker::checkMotionCommand(const msg::MotionCommand& command)
{
  SafetyCheckResult result;
  result.is_safe = true;
  result.error_code = 0;

  if (!safety_enabled_) {
    return result;
  }

  // 检查速度
  auto speed_result = checkSpeed(command.speed_ratio);
  if (!speed_result.is_safe) {
    return speed_result;
  }

  // 检查加速度
  if (command.acceleration_ratio < 0.0 || command.acceleration_ratio > max_acceleration_ratio_) {
    result.is_safe = false;
    result.error_message = "Acceleration ratio out of range: " + 
                          std::to_string(command.acceleration_ratio);
    result.error_code = -4;
    return result;
  }

  // 检查工作空间（如果是笛卡尔运动）
  if (command.motion_type == 1 || command.motion_type == 2) { // CARTESIAN_MOVE or LINEAR_MOVE
    auto workspace_result = checkWorkspace(command.target_pose);
    if (!workspace_result.is_safe) {
      return workspace_result;
    }
  }

  return result;
}

bool SafetyChecker::loadSafetyConfig(const std::string& config_file)
{
  try {
    YAML::Node config = YAML::LoadFile(config_file);

    // 加载工作空间限制
    if (config["workspace_limits"]) {
      auto limits = config["workspace_limits"];
      workspace_limits_.x_min = limits["x_min"].as<double>();
      workspace_limits_.x_max = limits["x_max"].as<double>();
      workspace_limits_.y_min = limits["y_min"].as<double>();
      workspace_limits_.y_max = limits["y_max"].as<double>();
      workspace_limits_.z_min = limits["z_min"].as<double>();
      workspace_limits_.z_max = limits["z_max"].as<double>();
    }

    // 加载速度限制
    if (config["max_speed_ratio"]) {
      max_speed_ratio_ = config["max_speed_ratio"].as<double>();
    }

    // 加载加速度限制
    if (config["max_acceleration_ratio"]) {
      max_acceleration_ratio_ = config["max_acceleration_ratio"].as<double>();
    }

    // 安全检查开关
    if (config["safety_enabled"]) {
      safety_enabled_ = config["safety_enabled"].as<bool>();
    }

    RCLCPP_INFO(logger_, "Safety configuration loaded from: %s", config_file.c_str());
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to load safety config: %s", e.what());
    return false;
  }
}

bool SafetyChecker::isInWorkspace(const geometry_msgs::msg::Pose& pose) const
{
  return (pose.position.x >= workspace_limits_.x_min && pose.position.x <= workspace_limits_.x_max &&
          pose.position.y >= workspace_limits_.y_min && pose.position.y <= workspace_limits_.y_max &&
          pose.position.z >= workspace_limits_.z_min && pose.position.z <= workspace_limits_.z_max);
}

void SafetyChecker::setWorkspaceLimits(const WorkspaceLimits& limits)
{
  workspace_limits_ = limits;
  RCLCPP_INFO(logger_, "Workspace limits updated");
}

void SafetyChecker::setMaxSpeed(double max_speed)
{
  max_speed_ratio_ = max_speed;
  RCLCPP_INFO(logger_, "Max speed ratio set to: %f", max_speed);
}

void SafetyChecker::setMaxAcceleration(double max_acceleration)
{
  max_acceleration_ratio_ = max_acceleration;
  RCLCPP_INFO(logger_, "Max acceleration ratio set to: %f", max_acceleration);
}

void SafetyChecker::enableSafetyCheck(bool enable)
{
  safety_enabled_ = enable;
  RCLCPP_INFO(logger_, "Safety check %s", enable ? "enabled" : "disabled");
}

bool SafetyChecker::isSafetyCheckEnabled() const
{
  return safety_enabled_;
}

} // namespace elu_robot_arm_framework
