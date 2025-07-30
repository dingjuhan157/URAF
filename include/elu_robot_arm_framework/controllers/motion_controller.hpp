/**
 * @file motion_controller.hpp
 * @brief 运动控制管理器
 * @author ELU Robotics Team
 * @date 2025
 */

#ifndef ELU_ROBOT_ARM_FRAMEWORK__CONTROLLERS__MOTION_CONTROLLER_HPP_
#define ELU_ROBOT_ARM_FRAMEWORK__CONTROLLERS__MOTION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include "elu_robot_arm_framework/interfaces/robot_arm_interface.hpp"
#include "elu_robot_arm_framework/utils/plugin_manager.hpp"
#include "elu_robot_arm_framework/safety/safety_checker.hpp"
#include "elu_robot_arm_framework/msg/motion_command.hpp"
#include "elu_robot_arm_framework/msg/robot_status.hpp"
#include "elu_robot_arm_framework/srv/switch_robot.hpp"
#include "elu_robot_arm_framework/srv/execute_motion.hpp"

namespace elu_robot_arm_framework
{

/**
 * @brief 运动控制管理器
 * 负责统一管理机械臂的运动控制
 */
class MotionController : public rclcpp::Node
{
public:
  explicit MotionController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~MotionController();

  // 机械臂管理
  bool loadRobot(const std::string& robot_id, const std::string& config_file);
  bool switchRobot(const std::string& robot_id);
  bool unloadRobot(const std::string& robot_id);
  std::string getCurrentRobotId() const;

  // 运动控制
  bool executeMotion(const msg::MotionCommand& command);
  bool emergencyStop();
  bool clearError();

  // 状态查询
  msg::RobotStatus getRobotStatus(const std::string& robot_id = "");
  std::vector<std::string> getLoadedRobots() const;

private:
  // 成员变量
  std::unique_ptr<PluginManager> plugin_manager_;
  std::unique_ptr<SafetyChecker> safety_checker_;
  std::unordered_map<std::string, std::shared_ptr<RobotArmInterface>> loaded_robots_;
  std::shared_ptr<RobotArmInterface> current_robot_;
  std::string current_robot_id_;

  // ROS2 接口
  rclcpp::Publisher<msg::RobotStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<msg::MotionCommand>::SharedPtr motion_subscriber_;
  rclcpp::Service<srv::SwitchRobot>::SharedPtr switch_robot_service_;
  rclcpp::Service<srv::ExecuteMotion>::SharedPtr execute_motion_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // 回调函数
  void motionCommandCallback(const msg::MotionCommand::SharedPtr msg);
  void switchRobotCallback(
    const std::shared_ptr<srv::SwitchRobot::Request> request,
    std::shared_ptr<srv::SwitchRobot::Response> response);
  void executeMotionCallback(
    const std::shared_ptr<srv::ExecuteMotion::Request> request,
    std::shared_ptr<srv::ExecuteMotion::Response> response);
  void statusTimerCallback();

  // 私有方法
  void loadRobotConfigs();
  bool validateMotionCommand(const msg::MotionCommand& command);
};

} // namespace elu_robot_arm_framework

#endif // ELU_ROBOT_ARM_FRAMEWORK__CONTROLLERS__MOTION_CONTROLLER_HPP_
