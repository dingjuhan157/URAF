#ifndef ELU_ROBOT_ARM_FRAMEWORK__ROKAE_MOTION_PLANNER_HPP_
#define ELU_ROBOT_ARM_FRAMEWORK__ROKAE_MOTION_PLANNER_HPP_

#include "elu_robot_arm_framework/interfaces/robot_arm_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <memory>
#include <vector>
#include <cmath>

namespace elu_robot_arm_framework
{

/**
 * @brief 路径点类型枚举
 */
enum class PathPointType {
  JOINT_SPACE,      ///< 关节空间路径点
  CARTESIAN_SPACE   ///< 笛卡尔空间路径点
};

/**
 * @brief 路径点结构
 */
struct PathPoint {
  PathPointType type;
  std::vector<double> joints;           ///< 关节角度 (弧度)
  geometry_msgs::msg::Pose pose;        ///< 笛卡尔位姿
  double speed_ratio;                   ///< 速度比例 [0.0, 1.0]
  double blend_radius;                  ///< 转弯区半径 (米)
  std::string frame_id;                 ///< 坐标系ID
  
  PathPoint() : type(PathPointType::JOINT_SPACE), speed_ratio(1.0), blend_radius(0.0) {}
};

/**
 * @brief 轨迹段结构
 */
struct TrajectorySegment {
  MotionType motion_type;
  PathPoint start_point;
  PathPoint end_point;
  PathPoint aux_point;                  ///< 辅助点 (圆弧运动用)
  double execution_time;                ///< 预估执行时间 (秒)
  
  TrajectorySegment() : motion_type(MotionType::JOINT_MOVE), execution_time(0.0) {}
};

/**
 * @brief 简化的DH参数结构
 */
struct DHParameter {
  double a;      ///< 连杆长度
  double d;      ///< 连杆偏移
  double alpha;  ///< 连杆扭转角
  double theta;  ///< 关节角度
  
  DHParameter(double a_val = 0.0, double d_val = 0.0, double alpha_val = 0.0, double theta_val = 0.0)
    : a(a_val), d(d_val), alpha(alpha_val), theta(theta_val) {}
};

/**
 * @brief Rokae运动规划器类
 * 
 * 提供机械臂运动规划功能，包括：
 * - 关节空间和笛卡尔空间轨迹规划
 * - 多段轨迹组合规划
 * - 碰撞检测和路径验证
 * - 速度和加速度优化
 */
class RokaeMotionPlanner
{
public:
  /**
   * @brief 构造函数
   * @param robot_adapter Rokae机械臂适配器指针
   */
  explicit RokaeMotionPlanner(std::shared_ptr<class RokaeAdapter> robot_adapter);
  
  /**
   * @brief 析构函数
   */
  ~RokaeMotionPlanner();

  // ==================== 单段轨迹规划 ====================
  
  /**
   * @brief 规划关节空间轨迹
   * @param start_joints 起始关节角度
   * @param target_joints 目标关节角度
   * @param speed_ratio 速度比例
   * @param trajectory 输出轨迹
   * @return 是否成功
   */
  bool planJointTrajectory(const std::vector<double>& start_joints,
                          const std::vector<double>& target_joints,
                          double speed_ratio,
                          trajectory_msgs::msg::JointTrajectory& trajectory);

  /**
   * @brief 规划笛卡尔直线轨迹  
   * @param start_pose 起始位姿
   * @param target_pose 目标位姿
   * @param speed_ratio 速度比例
   * @param trajectory 输出轨迹
   * @return 是否成功
   */
  bool planLinearTrajectory(const geometry_msgs::msg::Pose& start_pose,
                           const geometry_msgs::msg::Pose& target_pose,
                           double speed_ratio,
                           trajectory_msgs::msg::JointTrajectory& trajectory);

  /**
   * @brief 规划圆弧轨迹
   * @param start_pose 起始位姿
   * @param aux_pose 辅助点位姿
   * @param target_pose 目标位姿
   * @param speed_ratio 速度比例
   * @param trajectory 输出轨迹
   * @return 是否成功
   */
  bool planCircularTrajectory(const geometry_msgs::msg::Pose& start_pose,
                             const geometry_msgs::msg::Pose& aux_pose,
                             const geometry_msgs::msg::Pose& target_pose,
                             double speed_ratio,
                             trajectory_msgs::msg::JointTrajectory& trajectory);

  // ==================== 多段轨迹规划 ====================
  
  /**
   * @brief 规划多段组合轨迹
   * @param segments 轨迹段列表
   * @param trajectory 输出完整轨迹
   * @return 是否成功
   */
  bool planMultiSegmentTrajectory(const std::vector<TrajectorySegment>& segments,
                                 trajectory_msgs::msg::JointTrajectory& trajectory);

  /**
   * @brief 规划路径点序列轨迹
   * @param path_points 路径点列表
   * @param trajectory 输出轨迹
   * @return 是否成功
   */
  bool planPathPointsTrajectory(const std::vector<PathPoint>& path_points,
                               trajectory_msgs::msg::JointTrajectory& trajectory);

  // ==================== 轨迹验证和优化 ====================
  
  /**
   * @brief 验证轨迹可达性
   * @param trajectory 待验证轨迹
   * @return 是否可达
   */
  bool validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

  /**
   * @brief 检查路径碰撞
   * @param start_joints 起始关节角度
   * @param target_pose 目标位姿
   * @param target_joints 输出目标关节角度
   * @return 是否无碰撞
   */
  bool checkPathCollision(const std::vector<double>& start_joints,
                         const geometry_msgs::msg::Pose& target_pose,
                         std::vector<double>& target_joints);

  /**
   * @brief 优化轨迹时间
   * @param trajectory 输入轨迹
   * @param optimized_trajectory 输出优化轨迹
   * @return 是否成功
   */
  bool optimizeTrajectoryTiming(const trajectory_msgs::msg::JointTrajectory& trajectory,
                               trajectory_msgs::msg::JointTrajectory& optimized_trajectory);

  // ==================== 运动学计算 ====================
  
  /**
   * @brief 正运动学计算
   * @param joints 关节角度
   * @param pose 输出位姿
   * @return 是否成功
   */
  bool forwardKinematics(const std::vector<double>& joints,
                        geometry_msgs::msg::Pose& pose);

  /**
   * @brief 逆运动学计算
   * @param pose 目标位姿
   * @param seed_joints 种子关节角度
   * @param joints 输出关节角度
   * @return 是否成功
   */
  bool inverseKinematics(const geometry_msgs::msg::Pose& pose,
                        const std::vector<double>& seed_joints,
                        std::vector<double>& joints);

  // ==================== 参数设置 ====================
  
  /**
   * @brief 设置规划参数
   * @param max_velocity 最大速度比例
   * @param max_acceleration 最大加速度比例
   * @param planning_time_limit 规划时间限制 (秒)
   */
  void setPlanningParameters(double max_velocity = 1.0,
                            double max_acceleration = 1.0, 
                            double planning_time_limit = 5.0);

  /**
   * @brief 设置关节限制
   * @param joint_limits 关节限制 [下限, 上限] (弧度)
   */
  void setJointLimits(const std::vector<std::pair<double, double>>& joint_limits);

  /**
   * @brief 获取当前机械臂状态
   * @param current_joints 输出当前关节角度
   * @param current_pose 输出当前位姿
   * @return 是否成功
   */
  bool getCurrentState(std::vector<double>& current_joints,
                      geometry_msgs::msg::Pose& current_pose);

  // ==================== 新增实用功能 ====================
  
  /**
   * @brief 设置工作空间限制
   * @param workspace_limits 工作空间限制 [x_min, x_max, y_min, y_max, z_min, z_max]
   */
  void setWorkspaceLimits(const std::vector<double>& workspace_limits);
  
  /**
   * @brief 检查位姿是否在工作空间内
   * @param pose 目标位姿
   * @return 是否在工作空间内
   */
  bool isInWorkspace(const geometry_msgs::msg::Pose& pose);
  
  /**
   * @brief 计算两个关节配置之间的距离
   * @param joints1 关节配置1
   * @param joints2 关节配置2
   * @return 关节空间距离
   */
  double calculateJointSpaceDistance(const std::vector<double>& joints1,
                                    const std::vector<double>& joints2);

protected:
  /**
   * @brief 转换ROS轨迹为Rokae运动指令
   */
  bool convertTrajectoryToRokaeCommands(const trajectory_msgs::msg::JointTrajectory& trajectory);

  /**
   * @brief 插值计算轨迹点
   */
  bool interpolateTrajectoryPoints(const PathPoint& start, const PathPoint& end,
                                  int num_points, std::vector<PathPoint>& interpolated_points);

  /**
   * @brief 计算轨迹段时间
   */
  double calculateSegmentTime(const PathPoint& start, const PathPoint& end, double speed_ratio);

  /**
   * @brief 验证关节角度限制
   */
  bool validateJointLimits(const std::vector<double>& joints);

  /**
   * @brief 生成五次多项式轨迹
   */
  bool generateQuinticTrajectory(const std::vector<double>& start_joints,
                                const std::vector<double>& target_joints,
                                double duration,
                                int num_points,
                                trajectory_msgs::msg::JointTrajectory& trajectory);

  /**
   * @brief 简化的DH正运动学计算
   */
  geometry_msgs::msg::Pose calculateForwardKinematicsDH(const std::vector<double>& joints);

  /**
   * @brief 简化的数值逆运动学计算
   */
  bool calculateInverseKinematicsNumerical(const geometry_msgs::msg::Pose& target_pose,
                                          const std::vector<double>& seed_joints,
                                          std::vector<double>& result_joints);

  /**
   * @brief 轨迹平滑处理
   */
  bool smoothTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory);

private:
  std::shared_ptr<class RokaeAdapter> robot_adapter_;  ///< 机械臂适配器
  
  // 规划参数
  double max_velocity_;                 ///< 最大速度比例
  double max_acceleration_;            ///< 最大加速度比例  
  double planning_time_limit_;         ///< 规划时间限制
  
  // 关节限制
  std::vector<std::pair<double, double>> joint_limits_;  ///< 关节限制
  
  // 工作空间限制
  std::vector<double> workspace_limits_;  ///< 工作空间限制
  
  // DH参数 (Rokae机械臂的简化DH参数)
  std::vector<DHParameter> dh_params_;
  
  // 内部状态
  bool is_initialized_;                ///< 是否初始化
  std::vector<double> last_joints_;    ///< 上次关节角度
  
  // 数值计算参数
  static constexpr double IK_TOLERANCE = 1e-6;      ///< 逆运动学容差
  static constexpr int IK_MAX_ITERATIONS = 100;     ///< 逆运动学最大迭代次数
  static constexpr double COLLISION_THRESHOLD = 0.05; ///< 碰撞检测阈值
  
  // 日志
  rclcpp::Logger logger_;
  
  /**
   * @brief 初始化DH参数 (Rokae机械臂参数)
   */
  void initializeDHParameters();
  
  /**
   * @brief 矩阵乘法工具函数
   */
  std::vector<std::vector<double>> multiplyMatrices(const std::vector<std::vector<double>>& A,
                                                   const std::vector<std::vector<double>>& B);
  
  /**
   * @brief 创建DH变换矩阵
   */
  std::vector<std::vector<double>> createDHMatrix(const DHParameter& dh);
  
  /**
   * @brief 计算雅克比矩阵
   */
  std::vector<std::vector<double>> calculateJacobian(const std::vector<double>& joints);
};

} // namespace elu_robot_arm_framework

#endif // ELU_ROBOT_ARM_FRAMEWORK__ROKAE_MOTION_PLANNER_HPP_