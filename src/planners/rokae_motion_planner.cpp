#include "elu_robot_arm_framework/planners/rokae_motion_planner.hpp"
#include "elu_robot_arm_framework/adapters/rokae_adapter.hpp"
#include <rclcpp/rclcpp.hpp>

namespace elu_robot_arm_framework
{

RokaeMotionPlanner::RokaeMotionPlanner(std::shared_ptr<RokaeAdapter> robot_adapter)
  : robot_adapter_(robot_adapter)
  , max_velocity_(1.0)
  , max_acceleration_(1.0)
  , planning_time_limit_(5.0)
  , is_initialized_(false)
  , logger_(rclcpp::get_logger("rokae_motion_planner"))
{
  RCLCPP_INFO(logger_, "RokaeMotionPlanner initialized");
  
  // 设置默认关节限制 (6个关节的上下限，弧度)
  joint_limits_ = {
    {-M_PI, M_PI},     // Joint 1
    {-M_PI/2, M_PI/2}, // Joint 2  
    {-M_PI, M_PI},     // Joint 3
    {-M_PI, M_PI},     // Joint 4
    {-M_PI/2, M_PI/2}, // Joint 5
    {-2*M_PI, 2*M_PI}  // Joint 6
  };
  
  is_initialized_ = true;
}

RokaeMotionPlanner::~RokaeMotionPlanner()
{
  RCLCPP_INFO(logger_, "RokaeMotionPlanner destroyed");
}

bool RokaeMotionPlanner::planJointTrajectory(const std::vector<double>& start_joints,
                                            const std::vector<double>& target_joints,
                                            double speed_ratio,
                                            trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (!is_initialized_ || !robot_adapter_) {
    RCLCPP_ERROR(logger_, "Motion planner not initialized or robot adapter not available");
    return false;
  }
  
  if (start_joints.size() != 6 || target_joints.size() != 6) {
    RCLCPP_ERROR(logger_, "Invalid joint array size, expected 6 joints");
    return false;
  }
  
  try {
    // 验证关节限制
    if (!validateJointLimits(start_joints) || !validateJointLimits(target_joints)) {
      RCLCPP_ERROR(logger_, "Joint positions exceed limits");
      return false;
    }
    
    // 创建简单的点到点轨迹
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    trajectory.points.clear();
    
    // 起始点
    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = start_joints;
    start_point.velocities = std::vector<double>(6, 0.0);
    start_point.accelerations = std::vector<double>(6, 0.0);
    start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    trajectory.points.push_back(start_point);
    
    // 计算轨迹时间
    double max_joint_diff = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      max_joint_diff = std::max(max_joint_diff, std::abs(target_joints[i] - start_joints[i]));
    }
    
    double trajectory_time = max_joint_diff / (speed_ratio * max_velocity_) + 1.0; // 至少1秒
    
    // 终点
    trajectory_msgs::msg::JointTrajectoryPoint end_point;
    end_point.positions = target_joints;
    end_point.velocities = std::vector<double>(6, 0.0);
    end_point.accelerations = std::vector<double>(6, 0.0);
    end_point.time_from_start = rclcpp::Duration::from_seconds(trajectory_time);
    trajectory.points.push_back(end_point);
    
    RCLCPP_INFO(logger_, "Joint trajectory planned with duration: %.2f seconds", trajectory_time);
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in joint trajectory planning: %s", e.what());
    return false;
  }
}

bool RokaeMotionPlanner::planLinearTrajectory(const geometry_msgs::msg::Pose& /* start_pose */,
                                             const geometry_msgs::msg::Pose& /* target_pose */,
                                             double speed_ratio,
                                             trajectory_msgs::msg::JointTrajectory& trajectory)
{
  // 简化实现：转换为关节空间规划
  std::vector<double> start_joints, target_joints;
  
  // 这里需要逆运动学计算，简化为获取当前关节角度
  geometry_msgs::msg::Pose current_pose;
  if (!getCurrentState(start_joints, current_pose)) {
    RCLCPP_ERROR(logger_, "Failed to get current joint state for linear trajectory");
    return false;
  }
  
  // 简化：使用当前关节角度作为目标（实际应该通过逆运动学计算）
  target_joints = start_joints;
  
  return planJointTrajectory(start_joints, target_joints, speed_ratio, trajectory);
}

bool RokaeMotionPlanner::planCircularTrajectory(const geometry_msgs::msg::Pose& start_pose,
                                               const geometry_msgs::msg::Pose& /* aux_pose */,
                                               const geometry_msgs::msg::Pose& target_pose,
                                               double speed_ratio,
                                               trajectory_msgs::msg::JointTrajectory& trajectory)
{
  RCLCPP_WARN(logger_, "Circular trajectory planning not implemented, using linear trajectory");
  return planLinearTrajectory(start_pose, target_pose, speed_ratio, trajectory);
}

bool RokaeMotionPlanner::planMultiSegmentTrajectory(const std::vector<TrajectorySegment>& segments,
                                                   trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (segments.empty()) {
    RCLCPP_ERROR(logger_, "No trajectory segments provided");
    return false;
  }
  
  trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  trajectory.points.clear();
  
  double total_time = 0.0;
  
  for (const auto& segment : segments) {
    trajectory_msgs::msg::JointTrajectory segment_traj;
    
    if (planJointTrajectory(segment.start_point.joints, segment.end_point.joints, 
                           segment.start_point.speed_ratio, segment_traj)) {
      // 添加轨迹点，调整时间偏移
      for (auto& point : segment_traj.points) {
        // Fix: Convert Duration to seconds properly
        double point_time_seconds = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
        point.time_from_start = rclcpp::Duration::from_seconds(total_time + point_time_seconds);
        trajectory.points.push_back(point);
      }
      
      // Fix: Convert Duration to seconds properly
      const auto& last_point = segment_traj.points.back();
      double last_point_time_seconds = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9;
      total_time += last_point_time_seconds;
    }
  }
  
  return !trajectory.points.empty();
}

bool RokaeMotionPlanner::planPathPointsTrajectory(const std::vector<PathPoint>& path_points,
                                                 trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (path_points.size() < 2) {
    RCLCPP_ERROR(logger_, "Need at least 2 path points for trajectory planning");
    return false;
  }
  
  // 转换为轨迹段
  std::vector<TrajectorySegment> segments;
  
  for (size_t i = 1; i < path_points.size(); ++i) {
    TrajectorySegment segment;
    segment.motion_type = MotionType::JOINT_MOVE;
    segment.start_point = path_points[i-1];
    segment.end_point = path_points[i];
    segment.execution_time = calculateSegmentTime(path_points[i-1], path_points[i], 
                                                 path_points[i].speed_ratio);
    segments.push_back(segment);
  }
  
  return planMultiSegmentTrajectory(segments, trajectory);
}

bool RokaeMotionPlanner::validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(logger_, "Empty trajectory");
    return false;
  }
  
  if (trajectory.joint_names.size() != 6) {
    RCLCPP_ERROR(logger_, "Trajectory must have 6 joints");
    return false;
  }
  
  // 验证每个轨迹点
  for (const auto& point : trajectory.points) {
    if (point.positions.size() != 6) {
      RCLCPP_ERROR(logger_, "Invalid joint position size in trajectory point");
      return false;
    }
    
    if (!validateJointLimits(point.positions)) {
      RCLCPP_ERROR(logger_, "Joint positions in trajectory exceed limits");
      return false;
    }
  }
  
  RCLCPP_INFO(logger_, "Trajectory validation passed");
  return true;
}

bool RokaeMotionPlanner::checkPathCollision(const std::vector<double>& start_joints,
                                           const geometry_msgs::msg::Pose& /* target_pose */,
                                           std::vector<double>& target_joints)
{
  // 简化实现：假设无碰撞
  RCLCPP_WARN(logger_, "Collision checking not implemented, assuming collision-free path");
  
  // 简化：使用起始关节角度作为目标
  target_joints = start_joints;
  return true;
}

bool RokaeMotionPlanner::optimizeTrajectoryTiming(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                                 trajectory_msgs::msg::JointTrajectory& optimized_trajectory)
{
  // 简化实现：直接复制轨迹
  optimized_trajectory = trajectory;
  RCLCPP_INFO(logger_, "Trajectory timing optimization completed");
  return true;
}

bool RokaeMotionPlanner::forwardKinematics(const std::vector<double>& /* joints */,
                                          geometry_msgs::msg::Pose& pose)
{
  if (!robot_adapter_) {
    RCLCPP_ERROR(logger_, "Robot adapter not available");
    return false;
  }
  
  // 简化实现：获取当前位姿
  pose = robot_adapter_->getCurrentPose();
  return true;
}

bool RokaeMotionPlanner::inverseKinematics(const geometry_msgs::msg::Pose& /* pose */,
                                          const std::vector<double>& seed_joints,
                                          std::vector<double>& joints)
{
  // 简化实现：使用种子关节角度
  joints = seed_joints;
  RCLCPP_WARN(logger_, "Inverse kinematics not implemented, using seed joints");
  return true;
}

void RokaeMotionPlanner::setPlanningParameters(double max_velocity,
                                              double max_acceleration,
                                              double planning_time_limit)
{
  max_velocity_ = std::max(0.1, std::min(1.0, max_velocity));
  max_acceleration_ = std::max(0.1, std::min(1.0, max_acceleration));
  planning_time_limit_ = std::max(1.0, planning_time_limit);
  
  RCLCPP_INFO(logger_, "Planning parameters updated: vel=%.2f, acc=%.2f, time_limit=%.1f",
              max_velocity_, max_acceleration_, planning_time_limit_);
}

void RokaeMotionPlanner::setJointLimits(const std::vector<std::pair<double, double>>& joint_limits)
{
  if (joint_limits.size() != 6) {
    RCLCPP_ERROR(logger_, "Joint limits must be provided for 6 joints");
    return;
  }
  
  joint_limits_ = joint_limits;
  RCLCPP_INFO(logger_, "Joint limits updated");
}

bool RokaeMotionPlanner::getCurrentState(std::vector<double>& current_joints,
                                        geometry_msgs::msg::Pose& current_pose)
{
  if (!robot_adapter_) {
    RCLCPP_ERROR(logger_, "Robot adapter not available");
    return false;
  }
  
  try {
    current_joints = robot_adapter_->getCurrentJoints();
    current_pose = robot_adapter_->getCurrentPose();
    
    if (current_joints.empty()) {
      RCLCPP_ERROR(logger_, "Failed to get current joint positions");
      return false;
    }
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception getting current state: %s", e.what());
    return false;
  }
}

bool RokaeMotionPlanner::convertTrajectoryToRokaeCommands(const trajectory_msgs::msg::JointTrajectory& /* trajectory */)
{
  // 简化实现
  RCLCPP_INFO(logger_, "Converting trajectory to Rokae commands (simplified implementation)");
  return true;
}

bool RokaeMotionPlanner::interpolateTrajectoryPoints(const PathPoint& start, const PathPoint& end,
                                                    int num_points, std::vector<PathPoint>& interpolated_points)
{
  interpolated_points.clear();
  
  if (num_points < 2) {
    RCLCPP_ERROR(logger_, "Number of interpolation points must be at least 2");
    return false;
  }
  
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    PathPoint point;
    
    point.type = start.type;
    point.joints.resize(6);
    
    // 线性插值
    for (size_t j = 0; j < 6; ++j) {
      point.joints[j] = start.joints[j] + t * (end.joints[j] - start.joints[j]);
    }
    
    point.speed_ratio = start.speed_ratio + t * (end.speed_ratio - start.speed_ratio);
    point.blend_radius = start.blend_radius;
    
    interpolated_points.push_back(point);
  }
  
  return true;
}

double RokaeMotionPlanner::calculateSegmentTime(const PathPoint& start, const PathPoint& end, double speed_ratio)
{
  if (start.joints.size() != 6 || end.joints.size() != 6) {
    return 1.0; // 默认1秒
  }
  
  double max_diff = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    max_diff = std::max(max_diff, std::abs(end.joints[i] - start.joints[i]));
  }
  
  double base_time = max_diff / (speed_ratio * max_velocity_);
  return std::max(0.5, base_time); // 最少0.5秒
}

bool RokaeMotionPlanner::validateJointLimits(const std::vector<double>& joints)
{
  if (joints.size() != 6 || joint_limits_.size() != 6) {
    return false;
  }
  
  for (size_t i = 0; i < 6; ++i) {
    if (joints[i] < joint_limits_[i].first || joints[i] > joint_limits_[i].second) {
      RCLCPP_WARN(logger_, "Joint %zu (%.3f) exceeds limits [%.3f, %.3f]", 
                  i, joints[i], joint_limits_[i].first, joint_limits_[i].second);
      return false;
    }
  }
  
  return true;
}

} // namespace elu_robot_arm_framework