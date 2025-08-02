#ifndef ELU_ROBOT_ARM_FRAMEWORK__ROKAE_ADAPTER_HPP_
#define ELU_ROBOT_ARM_FRAMEWORK__ROKAE_ADAPTER_HPP_

#include "elu_robot_arm_framework/interfaces/robot_arm_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <atomic>

#ifdef XCORE_SDK_AVAILABLE
#include "rokae/robot.h"
#include "rokae/data_types.h"
#include "rokae/exception.h"
#include "rokae/utility.h"
#endif

namespace elu_robot_arm_framework
{

/**
 * @brief Rokae机械臂适配器类
 * 
 * 该类实现了与Rokae xMateCR7机械臂的通信接口，
 * 包括连接、运动控制、状态读取等功能
 */
class RokaeAdapter : public RobotArmInterface
{
public:
  RokaeAdapter();
  virtual ~RokaeAdapter();

  // 基础连接接口 - 修正方法签名
  bool connect(const std::string& config_file) override;
  bool disconnect() override;
  bool isConnected() const override;

  // 运动控制接口 - 添加缺失参数以匹配基类
  bool moveToJoint(const std::vector<double>& joints, double speed_ratio = 1.0) override;
  bool moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) override;
  bool linearMove(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) override;

  // 状态读取接口
  std::vector<double> getCurrentJoints() override;
  geometry_msgs::msg::Pose getCurrentPose() override;
  RobotState getStatus() override;
  std::string getErrorMessage() override;

  // 运动参数设置 - 修正方法名和参数
  bool setSpeed(double speed_ratio) override;
  bool setAcceleration(double acceleration_ratio) override;
  bool setPayload(double payload_kg) override;

  // 安全控制 - 修正方法名
  bool emergencyStop() override;
  bool clearError() override;
  bool enable() override;
  bool disable() override;

  // 信息获取
  std::string getRobotModel() const override;
  int getDoF() const override;
  double getMaxPayload() const override;
  std::vector<double> getJointLimits() const override;

  // 扩展接口（非基类要求的）- 移除override关键字
  bool moveToJointAsync(const std::vector<double>& joints);
  bool moveToPoseAsync(const geometry_msgs::msg::Pose& pose);
  bool setBlendRadius(double radius);
  bool setToolFrame(const geometry_msgs::msg::Pose& tool_frame);
  bool setWorkspaceFrame(const geometry_msgs::msg::Pose& workspace_frame);

protected:
  /**
   * @brief 从配置文件加载参数
   */
  bool loadConfiguration(const std::string& config_file);

  /**
   * @brief 初始化机械臂连接
   */
  bool initializeRobot();

  /**
   * @brief 等待机械臂运动完成
   */
  void waitForMotionComplete();

  /**
   * @brief 将ROS Pose转换为Rokae CartesianPosition
   */
  void rosePoseToRokaePosition(const geometry_msgs::msg::Pose& ros_pose, 
                               std::array<double, 6>& rokae_pose);

  /**
   * @brief 将Rokae CartesianPosition转换为ROS Pose
   */
  void rokaePositionToRosePose(const std::array<double, 6>& rokae_pose,
                               geometry_msgs::msg::Pose& ros_pose);

  /**
   * @brief 状态监控线程函数
   */
  void statusMonitoringThread();

private:
#ifdef XCORE_SDK_AVAILABLE
  std::unique_ptr<rokae::xMateRobot> robot_;  ///< Rokae机械臂对象
#endif

  // 连接参数
  std::string robot_ip_;              ///< 机械臂IP地址
  std::string local_ip_;              ///< 本机IP地址
  int connection_timeout_;            ///< 连接超时时间

  // 运动参数
  double default_speed_;              ///< 默认速度 (mm/s)
  double default_acceleration_;       ///< 默认加速度百分比
  double default_blend_radius_;       ///< 默认转弯区半径 (mm)

  // 状态管理
  std::atomic<bool> is_connected_;    ///< 连接状态
  std::atomic<bool> is_moving_;       ///< 运动状态
  std::atomic<bool> has_error_;       ///< 错误状态
  std::string last_error_message_;   ///< 最后错误信息

  // 线程管理
  std::thread status_monitor_thread_; ///< 状态监控线程
  std::atomic<bool> stop_monitoring_; ///< 停止监控标志

  // 日志
  rclcpp::Logger logger_;
};

} // namespace elu_robot_arm_framework

#endif // ELU_ROBOT_ARM_FRAMEWORK__ROKAE_ADAPTER_HPP_