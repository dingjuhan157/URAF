/**
 * @file robot_arm_interface.hpp
 * @brief 机械臂统一接口基类定义
 * @author ELU Robotics Team
 * @date 2025
 */

#ifndef ELU_ROBOT_ARM_FRAMEWORK__INTERFACES__ROBOT_ARM_INTERFACE_HPP_
#define ELU_ROBOT_ARM_FRAMEWORK__INTERFACES__ROBOT_ARM_INTERFACE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "elu_robot_arm_framework/msg/robot_status.hpp"

namespace elu_robot_arm_framework
{

/**
 * @brief 机械臂状态枚举
 */
enum class RobotState
{
  DISCONNECTED,
  CONNECTING,
  IDLE,
  MOVING,
  ERROR,
  EMERGENCY_STOP
};

/**
 * @brief 运动类型枚举
 */
enum class MotionType
{
  JOINT_MOVE,
  CARTESIAN_MOVE,
  LINEAR_MOVE,
  CIRCULAR_MOVE
};

/**
 * @brief 机械臂接口基类
 * 所有机械臂适配器都需要继承并实现此接口
 */
class RobotArmInterface
{
public:
  RobotArmInterface() = default;
  virtual ~RobotArmInterface() = default;

  // 连接管理
  virtual bool connect(const std::string& config_file) = 0;
  virtual bool disconnect() = 0;
  virtual bool isConnected() const = 0;

  // 运动控制
  virtual bool moveToJoint(const std::vector<double>& joints, double speed_ratio = 1.0) = 0;
  virtual bool moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) = 0;
  virtual bool linearMove(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) = 0;

  // 状态获取
  virtual std::vector<double> getCurrentJoints() = 0;
  virtual geometry_msgs::msg::Pose getCurrentPose() = 0;
  virtual RobotState getStatus() = 0;
  virtual std::string getErrorMessage() = 0;

  // 控制参数设置
  virtual bool setSpeed(double speed_ratio) = 0;
  virtual bool setAcceleration(double acceleration_ratio) = 0;
  virtual bool setPayload(double payload_kg) = 0;

  // 安全控制
  virtual bool emergencyStop() = 0;
  virtual bool clearError() = 0;
  virtual bool enable() = 0;
  virtual bool disable() = 0;

  // 信息获取
  virtual std::string getRobotModel() const = 0;
  virtual int getDoF() const = 0;
  virtual double getMaxPayload() const = 0;
  virtual std::vector<double> getJointLimits() const = 0;
};

} // namespace elu_robot_arm_framework

#endif // ELU_ROBOT_ARM_FRAMEWORK__INTERFACES__ROBOT_ARM_INTERFACE_HPP_
