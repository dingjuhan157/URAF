/**
 * @file elu_adapter.hpp
 * @brief ELU机械臂适配器
 * @author ELU Robotics Team
 * @date 2025
 */

#ifndef ELU_ADAPTER__ELU_ADAPTER_HPP_
#define ELU_ADAPTER__ELU_ADAPTER_HPP_

#include "elu_robot_arm_framework/interfaces/robot_arm_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace elu_adapter
{

/**
 * @brief ELU机械臂适配器实现
 */
class EluAdapter : public elu_robot_arm_framework::RobotArmInterface
{
public:
  EluAdapter();
  virtual ~EluAdapter();

  // 连接管理
  bool connect(const std::string& config_file) override;
  bool disconnect() override;
  bool isConnected() const override;

  // 运动控制
  bool moveToJoint(const std::vector<double>& joints, double speed_ratio = 1.0) override;
  bool moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) override;
  bool linearMove(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) override;

  // 状态获取
  std::vector<double> getCurrentJoints() override;
  geometry_msgs::msg::Pose getCurrentPose() override;
  elu_robot_arm_framework::RobotState getStatus() override;
  std::string getErrorMessage() override;

  // 控制参数设置
  bool setSpeed(double speed_ratio) override;
  bool setAcceleration(double acceleration_ratio) override;
  bool setPayload(double payload_kg) override;

  // 安全控制
  bool emergencyStop() override;
  bool clearError() override;
  bool enable() override;
  bool disable() override;

  // 信息获取
  std::string getRobotModel() const override;
  int getDoF() const override;
  double getMaxPayload() const override;
  std::vector<double> getJointLimits() const override;

private:
  // ELU SDK相关变量
  // TODO: 添加ELU SDK接口
  
  std::string ip_address_;
  int port_;
  bool connected_;
  elu_robot_arm_framework::RobotState current_state_;
  std::string error_message_;
  
  std::vector<double> current_joints_;
  geometry_msgs::msg::Pose current_pose_;
  double current_speed_ratio_;
  double current_acceleration_ratio_;
  double current_payload_;
  
  rclcpp::Logger logger_;

  // 私有方法
  bool initializeConnection();
  bool updateRobotState();
  void handleError(const std::string& error_msg);
};

} // namespace elu_adapter

#endif // ELU_ADAPTER__ELU_ADAPTER_HPP_
