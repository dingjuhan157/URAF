#include "elu_robot_arm_framework/planners/rokae_motion_planner.hpp"
#include "elu_robot_arm_framework/adapters/rokae_adapter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

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
  RCLCPP_INFO(logger_, "RokaeMotionPlanner initializing...");
  
  // 设置默认关节限制 (6个关节的上下限，弧度)
  joint_limits_ = {
    {-M_PI, M_PI},         // Joint 1: ±180°
    {-M_PI/2, M_PI/2},     // Joint 2: ±90°  
    {-M_PI, M_PI},         // Joint 3: ±180°
    {-M_PI, M_PI},         // Joint 4: ±180°
    {-M_PI/2, M_PI/2},     // Joint 5: ±90°
    {-2*M_PI, 2*M_PI}      // Joint 6: ±360°
  };
  
  // 设置默认工作空间限制 (米)
  workspace_limits_ = {
    -1.5, 1.5,  // x范围
    -1.5, 1.5,  // y范围
    -0.2, 2.0   // z范围
  };
  
  // 初始化DH参数
  initializeDHParameters();
  
  is_initialized_ = true;
  RCLCPP_INFO(logger_, "RokaeMotionPlanner initialized successfully");
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
  
  // 限制速度比例
  speed_ratio = std::clamp(speed_ratio, 0.1, 1.0);
  
  try {
    // 验证关节限制
    if (!validateJointLimits(start_joints) || !validateJointLimits(target_joints)) {
      RCLCPP_ERROR(logger_, "Joint positions exceed limits");
      return false;
    }
    
    // 计算轨迹时间
    double max_joint_diff = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      max_joint_diff = std::max(max_joint_diff, std::abs(target_joints[i] - start_joints[i]));
    }
    
    double duration = std::max(1.0, max_joint_diff / (speed_ratio * max_velocity_));
    
    // 生成五次多项式轨迹
    return generateQuinticTrajectory(start_joints, target_joints, duration, 50, trajectory);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in joint trajectory planning: %s", e.what());
    return false;
  }
}

bool RokaeMotionPlanner::planLinearTrajectory(const geometry_msgs::msg::Pose& start_pose,
                                             const geometry_msgs::msg::Pose& target_pose,
                                             double speed_ratio,
                                             trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (!is_initialized_) {
    RCLCPP_ERROR(logger_, "Motion planner not initialized");
    return false;
  }
  
  try {
    // 检查工作空间限制
    if (!isInWorkspace(start_pose) || !isInWorkspace(target_pose)) {
      RCLCPP_ERROR(logger_, "Poses outside workspace limits");
      return false;
    }
    
    // 获取当前关节角度作为种子
    std::vector<double> current_joints, start_joints, target_joints;
    geometry_msgs::msg::Pose current_pose;
    
    if (!getCurrentState(current_joints, current_pose)) {
      RCLCPP_ERROR(logger_, "Failed to get current state");
      return false;
    }
    
    // 计算起始和目标关节角度
    if (!inverseKinematics(start_pose, current_joints, start_joints)) {
      RCLCPP_ERROR(logger_, "Failed to solve inverse kinematics for start pose");
      return false;
    }
    
    if (!inverseKinematics(target_pose, start_joints, target_joints)) {
      RCLCPP_ERROR(logger_, "Failed to solve inverse kinematics for target pose");
      return false;
    }
    
    // 生成直线轨迹：在笛卡尔空间插值，然后转换为关节空间
    int num_segments = 20;
    std::vector<PathPoint> path_points;
    
    for (int i = 0; i <= num_segments; ++i) {
      double t = static_cast<double>(i) / num_segments;
      
      PathPoint point;
      point.type = PathPointType::CARTESIAN_SPACE;
      point.speed_ratio = speed_ratio;
      
      // 位置线性插值
      point.pose.position.x = start_pose.position.x + t * (target_pose.position.x - start_pose.position.x);
      point.pose.position.y = start_pose.position.y + t * (target_pose.position.y - start_pose.position.y);
      point.pose.position.z = start_pose.position.z + t * (target_pose.position.z - start_pose.position.z);
      
      // 方向球面线性插值（简化为线性插值）
      point.pose.orientation.x = start_pose.orientation.x + t * (target_pose.orientation.x - start_pose.orientation.x);
      point.pose.orientation.y = start_pose.orientation.y + t * (target_pose.orientation.y - start_pose.orientation.y);
      point.pose.orientation.z = start_pose.orientation.z + t * (target_pose.orientation.z - start_pose.orientation.z);
      point.pose.orientation.w = start_pose.orientation.w + t * (target_pose.orientation.w - start_pose.orientation.w);
      
      // 计算对应的关节角度
      std::vector<double> seed = (i == 0) ? start_joints : path_points.back().joints;
      if (!inverseKinematics(point.pose, seed, point.joints)) {
        RCLCPP_WARN(logger_, "Failed to solve IK for waypoint %d, using previous joints", i);
        point.joints = seed;
      }
      
      path_points.push_back(point);
    }
    
    return planPathPointsTrajectory(path_points, trajectory);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in linear trajectory planning: %s", e.what());
    return false;
  }
}

bool RokaeMotionPlanner::planCircularTrajectory(const geometry_msgs::msg::Pose& start_pose,
                                               const geometry_msgs::msg::Pose& aux_pose,
                                               const geometry_msgs::msg::Pose& target_pose,
                                               double speed_ratio,
                                               trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (!is_initialized_) {
    RCLCPP_ERROR(logger_, "Motion planner not initialized");
    return false;
  }
  
  try {
    // 检查工作空间限制
    if (!isInWorkspace(start_pose) || !isInWorkspace(aux_pose) || !isInWorkspace(target_pose)) {
      RCLCPP_ERROR(logger_, "Poses outside workspace limits");
      return false;
    }
    
    // 简化的圆弧轨迹：通过三点拟合圆弧
    std::vector<PathPoint> path_points;
    int num_segments = 30;
    
    // 获取当前关节角度作为种子
    std::vector<double> current_joints;
    geometry_msgs::msg::Pose current_pose;
    if (!getCurrentState(current_joints, current_pose)) {
      RCLCPP_ERROR(logger_, "Failed to get current state");
      return false;
    }
    
    for (int i = 0; i <= num_segments; ++i) {
      double t = static_cast<double>(i) / num_segments;
      
      PathPoint point;
      point.type = PathPointType::CARTESIAN_SPACE;
      point.speed_ratio = speed_ratio;
      
      // 简化的圆弧插值：使用二次贝塞尔曲线
      double t_inv = 1.0 - t;
      point.pose.position.x = t_inv * t_inv * start_pose.position.x + 
                             2 * t_inv * t * aux_pose.position.x + 
                             t * t * target_pose.position.x;
      point.pose.position.y = t_inv * t_inv * start_pose.position.y + 
                             2 * t_inv * t * aux_pose.position.y + 
                             t * t * target_pose.position.y;
      point.pose.position.z = t_inv * t_inv * start_pose.position.z + 
                             2 * t_inv * t * aux_pose.position.z + 
                             t * t * target_pose.position.z;
      
      // 方向插值
      if (t < 0.5) {
        double local_t = t * 2.0;
        point.pose.orientation.x = start_pose.orientation.x + local_t * (aux_pose.orientation.x - start_pose.orientation.x);
        point.pose.orientation.y = start_pose.orientation.y + local_t * (aux_pose.orientation.y - start_pose.orientation.y);
        point.pose.orientation.z = start_pose.orientation.z + local_t * (aux_pose.orientation.z - start_pose.orientation.z);
        point.pose.orientation.w = start_pose.orientation.w + local_t * (aux_pose.orientation.w - start_pose.orientation.w);
      } else {
        double local_t = (t - 0.5) * 2.0;
        point.pose.orientation.x = aux_pose.orientation.x + local_t * (target_pose.orientation.x - aux_pose.orientation.x);
        point.pose.orientation.y = aux_pose.orientation.y + local_t * (target_pose.orientation.y - aux_pose.orientation.y);
        point.pose.orientation.z = aux_pose.orientation.z + local_t * (target_pose.orientation.z - aux_pose.orientation.z);
        point.pose.orientation.w = aux_pose.orientation.w + local_t * (target_pose.orientation.w - aux_pose.orientation.w);
      }
      
      // 计算对应的关节角度
      std::vector<double> seed = (i == 0) ? current_joints : path_points.back().joints;
      if (!inverseKinematics(point.pose, seed, point.joints)) {
        RCLCPP_WARN(logger_, "Failed to solve IK for circular waypoint %d", i);
        point.joints = seed;
      }
      
      path_points.push_back(point);
    }
    
    return planPathPointsTrajectory(path_points, trajectory);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in circular trajectory planning: %s", e.what());
    return false;
  }
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
  
  for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
    const auto& segment = segments[seg_idx];
    trajectory_msgs::msg::JointTrajectory segment_traj;
    
    bool planning_success = false;
    
    switch (segment.motion_type) {
      case MotionType::JOINT_MOVE:
        planning_success = planJointTrajectory(segment.start_point.joints, 
                                             segment.end_point.joints, 
                                             segment.start_point.speed_ratio, 
                                             segment_traj);
        break;
        
      case MotionType::LINEAR_MOVE:
        planning_success = planLinearTrajectory(segment.start_point.pose, 
                                              segment.end_point.pose, 
                                              segment.start_point.speed_ratio, 
                                              segment_traj);
        break;
        
      case MotionType::CIRCULAR_MOVE:
        planning_success = planCircularTrajectory(segment.start_point.pose, 
                                                segment.aux_point.pose,
                                                segment.end_point.pose, 
                                                segment.start_point.speed_ratio, 
                                                segment_traj);
        break;
        
      default:
        RCLCPP_ERROR(logger_, "Unsupported motion type in segment %zu", seg_idx);
        continue;
    }
    
    if (planning_success && !segment_traj.points.empty()) {
      // 添加轨迹点，调整时间偏移
      for (auto& point : segment_traj.points) {
        if (seg_idx > 0 || point.time_from_start.sec > 0 || point.time_from_start.nanosec > 0) {
          double point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
          point.time_from_start = rclcpp::Duration::from_seconds(total_time + point_time);
          trajectory.points.push_back(point);
        }
      }
      
      // 更新总时间
      const auto& last_point = segment_traj.points.back();
      double segment_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9;
      total_time = segment_duration;
    } else {
      RCLCPP_ERROR(logger_, "Failed to plan segment %zu", seg_idx);
      return false;
    }
  }
  
  // 平滑处理
  if (!trajectory.points.empty()) {
    smoothTrajectory(trajectory);
  }
  
  RCLCPP_INFO(logger_, "Multi-segment trajectory planned with %zu points, duration: %.2f seconds", 
              trajectory.points.size(), total_time);
  
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
    
    // 根据路径点类型确定运动类型
    if (path_points[i-1].type == PathPointType::JOINT_SPACE && 
        path_points[i].type == PathPointType::JOINT_SPACE) {
      segment.motion_type = MotionType::JOINT_MOVE;
    } else {
      segment.motion_type = MotionType::LINEAR_MOVE;
    }
    
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
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto& point = trajectory.points[i];
    
    if (point.positions.size() != 6) {
      RCLCPP_ERROR(logger_, "Invalid joint position size in trajectory point %zu", i);
      return false;
    }
    
    if (!validateJointLimits(point.positions)) {
      RCLCPP_ERROR(logger_, "Joint positions in trajectory point %zu exceed limits", i);
      return false;
    }
    
    // 检查速度和加速度限制
    if (!point.velocities.empty() && point.velocities.size() != 6) {
      RCLCPP_ERROR(logger_, "Invalid velocity size in trajectory point %zu", i);
      return false;
    }
    
    if (!point.accelerations.empty() && point.accelerations.size() != 6) {
      RCLCPP_ERROR(logger_, "Invalid acceleration size in trajectory point %zu", i);
      return false;
    }
    
    // 检查时间单调性
    if (i > 0) {
      double prev_time = trajectory.points[i-1].time_from_start.sec + 
                        trajectory.points[i-1].time_from_start.nanosec / 1e9;
      double curr_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
      
      if (curr_time <= prev_time) {
        RCLCPP_ERROR(logger_, "Non-monotonic time in trajectory at point %zu", i);
        return false;
      }
    }
  }
  
  RCLCPP_INFO(logger_, "Trajectory validation passed with %zu points", trajectory.points.size());
  return true;
}

bool RokaeMotionPlanner::checkPathCollision(const std::vector<double>& start_joints,
                                           const geometry_msgs::msg::Pose& target_pose,
                                           std::vector<double>& target_joints)
{
  try {
    // 简化的碰撞检测：检查工作空间限制
    if (!isInWorkspace(target_pose)) {
      RCLCPP_WARN(logger_, "Target pose outside workspace limits");
      return false;
    }
    
    // 计算目标关节角度
    if (!inverseKinematics(target_pose, start_joints, target_joints)) {
      RCLCPP_WARN(logger_, "Failed to solve inverse kinematics for collision check");
      return false;
    }
    
    // 检查关节限制
    if (!validateJointLimits(target_joints)) {
      RCLCPP_WARN(logger_, "Target joints exceed limits");
      return false;
    }
    
    // 简单的自碰撞检测：检查关节角度变化是否过大
    double max_joint_change = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      max_joint_change = std::max(max_joint_change, 
                                 std::abs(target_joints[i] - start_joints[i]));
    }
    
    if (max_joint_change > M_PI) {
      RCLCPP_WARN(logger_, "Large joint movement detected, potential collision risk");
      return false;
    }
    
    RCLCPP_DEBUG(logger_, "Path collision check passed");
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in collision checking: %s", e.what());
    return false;
  }
}

bool RokaeMotionPlanner::optimizeTrajectoryTiming(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                                 trajectory_msgs::msg::JointTrajectory& optimized_trajectory)
{
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(logger_, "Empty trajectory for optimization");
    return false;
  }
  
  optimized_trajectory = trajectory;
  
  try {
    // 简化的时间优化：重新计算每段的时间
    for (size_t i = 1; i < optimized_trajectory.points.size(); ++i) {
      auto& prev_point = optimized_trajectory.points[i-1];
      auto& curr_point = optimized_trajectory.points[i];
      
      // 计算关节变化量
      double max_joint_diff = 0.0;
      for (size_t j = 0; j < 6; ++j) {
        max_joint_diff = std::max(max_joint_diff, 
                                 std::abs(curr_point.positions[j] - prev_point.positions[j]));
      }
      
      // 重新计算时间
      double prev_time = prev_point.time_from_start.sec + prev_point.time_from_start.nanosec / 1e9;
      double optimal_duration = std::max(0.1, max_joint_diff / max_velocity_);
      curr_point.time_from_start = rclcpp::Duration::from_seconds(prev_time + optimal_duration);
    }
    
    // 重新计算速度和加速度
    for (size_t i = 0; i < optimized_trajectory.points.size(); ++i) {
      auto& point = optimized_trajectory.points[i];
      
      point.velocities.resize(6, 0.0);
      point.accelerations.resize(6, 0.0);
      
      if (i > 0 && i < optimized_trajectory.points.size() - 1) {
        const auto& prev_point = optimized_trajectory.points[i-1];
        const auto& next_point = optimized_trajectory.points[i+1];
        
        double dt1 = (point.time_from_start.sec + point.time_from_start.nanosec / 1e9) -
                     (prev_point.time_from_start.sec + prev_point.time_from_start.nanosec / 1e9);
        double dt2 = (next_point.time_from_start.sec + next_point.time_from_start.nanosec / 1e9) -
                     (point.time_from_start.sec + point.time_from_start.nanosec / 1e9);
        
        for (size_t j = 0; j < 6; ++j) {
          // 中心差分计算速度
          if (dt1 > 0 && dt2 > 0) {
            point.velocities[j] = (next_point.positions[j] - prev_point.positions[j]) / (dt1 + dt2);
          }
        }
      }
    }
    
    RCLCPP_INFO(logger_, "Trajectory timing optimization completed");
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in trajectory optimization: %s", e.what());
    return false;
  }
}

bool RokaeMotionPlanner::forwardKinematics(const std::vector<double>& joints,
                                          geometry_msgs::msg::Pose& pose)
{
  if (joints.size() != 6) {
    RCLCPP_ERROR(logger_, "Invalid joint array size for forward kinematics");
    return false;
  }
  
  try {
    pose = calculateForwardKinematicsDH(joints);
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in forward kinematics: %s", e.what());
    return false;
  }
}

bool RokaeMotionPlanner::inverseKinematics(const geometry_msgs::msg::Pose& pose,
                                          const std::vector<double>& seed_joints,
                                          std::vector<double>& joints)
{
  if (seed_joints.size() != 6) {
    RCLCPP_ERROR(logger_, "Invalid seed joint array size for inverse kinematics");
    return false;
  }
  
  try {
    return calculateInverseKinematicsNumerical(pose, seed_joints, joints);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in inverse kinematics: %s", e.what());
    return false;
  }
}

void RokaeMotionPlanner::setPlanningParameters(double max_velocity,
                                              double max_acceleration,
                                              double planning_time_limit)
{
  max_velocity_ = std::clamp(max_velocity, 0.1, 2.0);
  max_acceleration_ = std::clamp(max_acceleration, 0.1, 2.0);
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

void RokaeMotionPlanner::setWorkspaceLimits(const std::vector<double>& workspace_limits)
{
  if (workspace_limits.size() != 6) {
    RCLCPP_ERROR(logger_, "Workspace limits must have 6 values [x_min, x_max, y_min, y_max, z_min, z_max]");
    return;
  }
  
  workspace_limits_ = workspace_limits;
  RCLCPP_INFO(logger_, "Workspace limits updated");
}

bool RokaeMotionPlanner::isInWorkspace(const geometry_msgs::msg::Pose& pose)
{
  if (workspace_limits_.size() != 6) {
    return true; // 未设置工作空间限制
  }
  
  return (pose.position.x >= workspace_limits_[0] && pose.position.x <= workspace_limits_[1] &&
          pose.position.y >= workspace_limits_[2] && pose.position.y <= workspace_limits_[3] &&
          pose.position.z >= workspace_limits_[4] && pose.position.z <= workspace_limits_[5]);
}

double RokaeMotionPlanner::calculateJointSpaceDistance(const std::vector<double>& joints1,
                                                      const std::vector<double>& joints2)
{
  if (joints1.size() != 6 || joints2.size() != 6) {
    return std::numeric_limits<double>::max();
  }
  
  double distance = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    double diff = joints2[i] - joints1[i];
    distance += diff * diff;
  }
  
  return std::sqrt(distance);
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
    
    last_joints_ = current_joints;  // 保存最后的关节角度
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception getting current state: %s", e.what());
    return false;
  }
}

// ==================== Protected Methods ====================

bool RokaeMotionPlanner::convertTrajectoryToRokaeCommands(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(logger_, "Empty trajectory for conversion");
    return false;
  }
  
  RCLCPP_INFO(logger_, "Converting ROS trajectory with %zu points to Rokae commands", 
              trajectory.points.size());
  
  // 这里可以添加实际的Rokae SDK调用
  // 例如：robot_adapter_->executeTrajectory(trajectory);
  
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
    
    // 线性插值关节角度
    for (size_t j = 0; j < 6; ++j) {
      point.joints[j] = start.joints[j] + t * (end.joints[j] - start.joints[j]);
    }
    
    // 插值位姿
    point.pose.position.x = start.pose.position.x + t * (end.pose.position.x - start.pose.position.x);
    point.pose.position.y = start.pose.position.y + t * (end.pose.position.y - start.pose.position.y);
    point.pose.position.z = start.pose.position.z + t * (end.pose.position.z - start.pose.position.z);
    
    // 四元数插值（简化）
    point.pose.orientation.x = start.pose.orientation.x + t * (end.pose.orientation.x - start.pose.orientation.x);
    point.pose.orientation.y = start.pose.orientation.y + t * (end.pose.orientation.y - start.pose.orientation.y);
    point.pose.orientation.z = start.pose.orientation.z + t * (end.pose.orientation.z - start.pose.orientation.z);
    point.pose.orientation.w = start.pose.orientation.w + t * (end.pose.orientation.w - start.pose.orientation.w);
    
    point.speed_ratio = start.speed_ratio + t * (end.speed_ratio - start.speed_ratio);
    point.blend_radius = start.blend_radius;
    point.frame_id = start.frame_id;
    
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
  return std::max(0.2, base_time); // 最少0.2秒
}

bool RokaeMotionPlanner::validateJointLimits(const std::vector<double>& joints)
{
  if (joints.size() != 6 || joint_limits_.size() != 6) {
    return false;
  }
  
  for (size_t i = 0; i < 6; ++i) {
    if (joints[i] < joint_limits_[i].first || joints[i] > joint_limits_[i].second) {
      RCLCPP_WARN(logger_, "Joint %zu (%.3f rad) exceeds limits [%.3f, %.3f]", 
                  i, joints[i], joint_limits_[i].first, joint_limits_[i].second);
      return false;
    }
  }
  
  return true;
}

bool RokaeMotionPlanner::generateQuinticTrajectory(const std::vector<double>& start_joints,
                                                  const std::vector<double>& target_joints,
                                                  double duration,
                                                  int num_points,
                                                  trajectory_msgs::msg::JointTrajectory& trajectory)
{
  trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  trajectory.points.clear();
  
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1) * duration;
    double s = t / duration;  // 归一化时间 [0, 1]
    
    // 五次多项式系数 (确保0速度和0加速度边界条件)
    double s3 = s * s * s;
    double s4 = s3 * s;
    double s5 = s4 * s;
    double poly = 10 * s3 - 15 * s4 + 6 * s5;
    double poly_dot = (30 * s * s - 60 * s3 + 30 * s4) / duration;
    double poly_ddot = (60 * s - 180 * s * s + 120 * s3) / (duration * duration);
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(6);
    point.velocities.resize(6);
    point.accelerations.resize(6);
    
    for (size_t j = 0; j < 6; ++j) {
      double joint_diff = target_joints[j] - start_joints[j];
      point.positions[j] = start_joints[j] + poly * joint_diff;
      point.velocities[j] = poly_dot * joint_diff;
      point.accelerations[j] = poly_ddot * joint_diff;
    }
    
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    trajectory.points.push_back(point);
  }
  
  RCLCPP_INFO(logger_, "Generated quintic trajectory with %d points, duration: %.2f seconds", 
              num_points, duration);
  return true;
}

geometry_msgs::msg::Pose RokaeMotionPlanner::calculateForwardKinematicsDH(const std::vector<double>& joints)
{
  geometry_msgs::msg::Pose pose;
  
  if (dh_params_.size() != 6 || joints.size() != 6) {
    RCLCPP_ERROR(logger_, "Invalid DH parameters or joint configuration");
    return pose;
  }
  
  // 创建单位变换矩阵
  std::vector<std::vector<double>> T = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };
  
  // 累乘DH变换矩阵
  for (size_t i = 0; i < 6; ++i) {
    DHParameter dh = dh_params_[i];
    dh.theta += joints[i];  // 添加关节角度
    
    auto Ti = createDHMatrix(dh);
    T = multiplyMatrices(T, Ti);
  }
  
  // 提取位置
  pose.position.x = T[0][3];
  pose.position.y = T[1][3];
  pose.position.z = T[2][3];
  
  // 提取旋转（简化的旋转矩阵到四元数转换）
  double trace = T[0][0] + T[1][1] + T[2][2];
  if (trace > 0) {
    double s = std::sqrt(trace + 1.0) * 2;
    pose.orientation.w = 0.25 * s;
    pose.orientation.x = (T[2][1] - T[1][2]) / s;
    pose.orientation.y = (T[0][2] - T[2][0]) / s;
    pose.orientation.z = (T[1][0] - T[0][1]) / s;
  } else if (T[0][0] > T[1][1] && T[0][0] > T[2][2]) {
    double s = std::sqrt(1.0 + T[0][0] - T[1][1] - T[2][2]) * 2;
    pose.orientation.w = (T[2][1] - T[1][2]) / s;
    pose.orientation.x = 0.25 * s;
    pose.orientation.y = (T[0][1] + T[1][0]) / s;
    pose.orientation.z = (T[0][2] + T[2][0]) / s;
  } else if (T[1][1] > T[2][2]) {
    double s = std::sqrt(1.0 + T[1][1] - T[0][0] - T[2][2]) * 2;
    pose.orientation.w = (T[0][2] - T[2][0]) / s;
    pose.orientation.x = (T[0][1] + T[1][0]) / s;
    pose.orientation.y = 0.25 * s;
    pose.orientation.z = (T[1][2] + T[2][1]) / s;
  } else {
    double s = std::sqrt(1.0 + T[2][2] - T[0][0] - T[1][1]) * 2;
    pose.orientation.w = (T[1][0] - T[0][1]) / s;
    pose.orientation.x = (T[0][2] + T[2][0]) / s;
    pose.orientation.y = (T[1][2] + T[2][1]) / s;
    pose.orientation.z = 0.25 * s;
  }
  
  return pose;
}

bool RokaeMotionPlanner::calculateInverseKinematicsNumerical(const geometry_msgs::msg::Pose& target_pose,
                                                           const std::vector<double>& seed_joints,
                                                           std::vector<double>& result_joints)
{
  result_joints = seed_joints;
  
  for (int iter = 0; iter < IK_MAX_ITERATIONS; ++iter) {
    // 计算当前位姿
    geometry_msgs::msg::Pose current_pose = calculateForwardKinematicsDH(result_joints);
    
    // 计算位置误差
    double dx = target_pose.position.x - current_pose.position.x;
    double dy = target_pose.position.y - current_pose.position.y;
    double dz = target_pose.position.z - current_pose.position.z;
    
    // 简化的方向误差计算
    double dqx = target_pose.orientation.x - current_pose.orientation.x;
    double dqy = target_pose.orientation.y - current_pose.orientation.y;
    double dqz = target_pose.orientation.z - current_pose.orientation.z;
    
    // 检查收敛
    double pos_error = std::sqrt(dx*dx + dy*dy + dz*dz);
    double ori_error = std::sqrt(dqx*dqx + dqy*dqy + dqz*dqz);
    
    if (pos_error < IK_TOLERANCE && ori_error < IK_TOLERANCE) {
      RCLCPP_DEBUG(logger_, "IK converged in %d iterations", iter);
      return true;
    }
    
    // 计算雅克比矩阵
    auto jacobian = calculateJacobian(result_joints);
    
    // 简化的雅克比逆运算（伪逆）
    std::vector<double> delta_joints(6, 0.0);
    double damping = 0.1;
    
    for (size_t i = 0; i < 6; ++i) {
      delta_joints[i] = (jacobian[0][i] * dx + jacobian[1][i] * dy + jacobian[2][i] * dz +
                        jacobian[3][i] * dqx + jacobian[4][i] * dqy + jacobian[5][i] * dqz) * damping;
    }
    
    // 更新关节角度
    for (size_t i = 0; i < 6; ++i) {
      result_joints[i] += delta_joints[i];
      
      // 限制关节范围
      result_joints[i] = std::clamp(result_joints[i], 
                                   joint_limits_[i].first, 
                                   joint_limits_[i].second);
    }
  }
  
  RCLCPP_WARN(logger_, "IK failed to converge after %d iterations", IK_MAX_ITERATIONS);
  return false;
}

bool RokaeMotionPlanner::smoothTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.size() < 3) {
    return true; // 点太少，无需平滑
  }
  
  // 简单的移动平均滤波
  auto original_points = trajectory.points;
  
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      // 位置平滑
      trajectory.points[i].positions[j] = 0.25 * original_points[i-1].positions[j] +
                                         0.5 * original_points[i].positions[j] +
                                         0.25 * original_points[i+1].positions[j];
      
      // 速度平滑
      if (!trajectory.points[i].velocities.empty()) {
        trajectory.points[i].velocities[j] = 0.25 * original_points[i-1].velocities[j] +
                                           0.5 * original_points[i].velocities[j] +
                                           0.25 * original_points[i+1].velocities[j];
      }
    }
  }
  
  return true;
}

// ==================== Private Methods ====================

void RokaeMotionPlanner::initializeDHParameters()
{
  // Rokae机械臂的简化DH参数 (实际参数需要根据具体型号调整)
  dh_params_ = {
    DHParameter(0.0,    0.333,  0.0,      0.0),    // Joint 1
    DHParameter(0.0,    0.0,   -M_PI/2,   0.0),    // Joint 2
    DHParameter(0.0,    0.316,  M_PI/2,   0.0),    // Joint 3
    DHParameter(0.0825, 0.0,   -M_PI/2,   0.0),    // Joint 4
    DHParameter(-0.0825,0.384, M_PI/2,   0.0),    // Joint 5
    DHParameter(0.0,    0.0,   -M_PI/2,   0.0)     // Joint 6
  };
  
  RCLCPP_INFO(logger_, "DH parameters initialized for Rokae robot");
}

std::vector<std::vector<double>> RokaeMotionPlanner::multiplyMatrices(
  const std::vector<std::vector<double>>& A,
  const std::vector<std::vector<double>>& B)
{
  std::vector<std::vector<double>> C(4, std::vector<double>(4, 0.0));
  
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  
  return C;
}

std::vector<std::vector<double>> RokaeMotionPlanner::createDHMatrix(const DHParameter& dh)
{
  double ct = std::cos(dh.theta);
  double st = std::sin(dh.theta);
  double ca = std::cos(dh.alpha);
  double sa = std::sin(dh.alpha);
  
  return {
    {ct,      -st*ca,    st*sa,     dh.a*ct},
    {st,       ct*ca,   -ct*sa,     dh.a*st},
    {0.0,      sa,       ca,        dh.d   },
    {0.0,      0.0,      0.0,       1.0    }
  };
}

std::vector<std::vector<double>> RokaeMotionPlanner::calculateJacobian(const std::vector<double>& joints)
{
  std::vector<std::vector<double>> jacobian(6, std::vector<double>(6, 0.0));
  
  const double delta = 1e-6;
  
  geometry_msgs::msg::Pose pose_center = calculateForwardKinematicsDH(joints);
  
  for (size_t i = 0; i < 6; ++i) {
    std::vector<double> joints_plus = joints;
    joints_plus[i] += delta;
    
    geometry_msgs::msg::Pose pose_plus = calculateForwardKinematicsDH(joints_plus);
    
    // 位置雅克比
    jacobian[0][i] = (pose_plus.position.x - pose_center.position.x) / delta;
    jacobian[1][i] = (pose_plus.position.y - pose_center.position.y) / delta;
    jacobian[2][i] = (pose_plus.position.z - pose_center.position.z) / delta;
    
    // 方向雅克比 (简化)
    jacobian[3][i] = (pose_plus.orientation.x - pose_center.orientation.x) / delta;
    jacobian[4][i] = (pose_plus.orientation.y - pose_center.orientation.y) / delta;
    jacobian[5][i] = (pose_plus.orientation.z - pose_center.orientation.z) / delta;
  }
  
  return jacobian;
}

} // namespace elu_robot_arm_framework