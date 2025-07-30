/**
 * @file safety_checker.hpp
 * @brief 安全检查器
 * @author ELU Robotics Team
 * @date 2025
 */

#ifndef ELU_ROBOT_ARM_FRAMEWORK__SAFETY__SAFETY_CHECKER_HPP_
#define ELU_ROBOT_ARM_FRAMEWORK__SAFETY__SAFETY_CHECKER_HPP_

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "elu_robot_arm_framework/msg/motion_command.hpp"

namespace elu_robot_arm_framework
{

/**
 * @brief 安全检查结果
 */
struct SafetyCheckResult
{
  bool is_safe;
  std::string error_message;
  int error_code;
};

/**
 * @brief 工作空间限制
 */
struct WorkspaceLimits
{
  double x_min, x_max;
  double y_min, y_max;
  double z_min, z_max;
};

/**
 * @brief 安全检查器
 * 负责机械臂运动的安全检查
 */
class SafetyChecker
{
public:
  explicit SafetyChecker(const std::string& config_file = "");
  virtual ~SafetyChecker() = default;

  // 安全检查
  SafetyCheckResult checkJointLimits(const std::vector<double>& joints, 
                                   const std::vector<double>& joint_limits);
  SafetyCheckResult checkWorkspace(const geometry_msgs::msg::Pose& pose);
  SafetyCheckResult checkSpeed(double speed_ratio);
  SafetyCheckResult checkMotionCommand(const msg::MotionCommand& command);

  // 配置管理
  bool loadSafetyConfig(const std::string& config_file);
  void setWorkspaceLimits(const WorkspaceLimits& limits);
  void setMaxSpeed(double max_speed);
  void setMaxAcceleration(double max_acceleration);

  // 状态管理
  void enableSafetyCheck(bool enable);
  bool isSafetyCheckEnabled() const;

private:
  WorkspaceLimits workspace_limits_;
  double max_speed_ratio_;
  double max_acceleration_ratio_;
  bool safety_enabled_;
  
  rclcpp::Logger logger_;

  // 私有方法
  bool isInWorkspace(const geometry_msgs::msg::Pose& pose) const;
  bool areJointsInLimits(const std::vector<double>& joints, 
                        const std::vector<double>& limits) const;
};

} // namespace elu_robot_arm_framework

#endif // ELU_ROBOT_ARM_FRAMEWORK__SAFETY__SAFETY_CHECKER_HPP_
