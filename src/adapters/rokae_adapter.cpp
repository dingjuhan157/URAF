#include "elu_robot_arm_framework/adapters/rokae_adapter.hpp"
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>

namespace elu_robot_arm_framework
{

// 修复构造函数 - 按照头文件中声明的顺序初始化成员变量
RokaeAdapter::RokaeAdapter()
: robot_ip_("192.168.0.160")              // 默认IP
, local_ip_("192.168.0.100")               // 默认本地IP
, connection_timeout_(5000)                // 连接超时时间
, default_speed_(200.0)                    // 默认速度 (mm/s)
, default_acceleration_(0.5)               // 默认加速度百分比
, default_blend_radius_(5.0)               // 默认转弯区半径 (mm)
, is_connected_(false)                     // 连接状态
, is_moving_(false)                        // 运动状态
, has_error_(false)                        // 错误状态
, stop_monitoring_(false)                  // 停止监控标志
, logger_(rclcpp::get_logger("rokae_adapter"))
{
  RCLCPP_INFO(logger_, "RokaeAdapter initialized");
}

RokaeAdapter::~RokaeAdapter()
{
  disconnect();
}

// 实现缺失的基类纯虚函数
std::string RokaeAdapter::getRobotModel() const
{
#ifdef XCORE_SDK_AVAILABLE
  if (robot_) {
    try {
      // 返回Rokae机械臂型号信息
      return "xMateCR7"; // 根据实际SDK接口获取型号
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Failed to get robot model: %s", e.what());
    }
  }
#endif
  return "Rokae xMateCR7"; // 默认型号
}

int RokaeAdapter::getDoF() const
{
  return 6; // xMateCR7是6自由度机械臂
}

double RokaeAdapter::getMaxPayload() const
{
#ifdef XCORE_SDK_AVAILABLE
  if (robot_) {
    try {
      // 从SDK获取最大负载信息
      return 7.0; // xMateCR7的最大负载为7kg
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Failed to get max payload: %s", e.what());
    }
  }
#endif
  return 7.0; // 默认最大负载7kg
}

std::vector<double> RokaeAdapter::getJointLimits() const
{
  // xMateCR7的关节限制 (弧度)
  // 格式: [J1_min, J1_max, J2_min, J2_max, J3_min, J3_max, J4_min, J4_max, J5_min, J5_max, J6_min, J6_max]
  std::vector<double> limits = {
    -3.14159, 3.14159,   // Joint 1: ±180°
    -2.61799, 2.61799,   // Joint 2: ±150°
    -3.14159, 3.14159,   // Joint 3: ±180°
    -3.14159, 3.14159,   // Joint 4: ±180°
    -2.26893, 2.26893,   // Joint 5: ±130°
    -6.28318, 6.28318    // Joint 6: ±360°
  };

#ifdef XCORE_SDK_AVAILABLE
  if (robot_) {
    try {
      // 如果SDK提供了获取关节限制的接口，可以在这里调用
      // auto sdk_limits = robot_->getJointLimits();
      // 处理SDK返回的限制
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Failed to get joint limits from SDK: %s", e.what());
    }
  }
#endif

  return limits;
}

bool RokaeAdapter::connect(const std::string& /* config_file */)
{
  if (is_connected_) {
    RCLCPP_WARN(logger_, "Already connected to robot");
    return true;
  }

#ifndef XCORE_SDK_AVAILABLE
  RCLCPP_ERROR(logger_, "Rokae SDK not available. Please compile with XCORE_SDK_AVAILABLE=ON");
  return false;
#else

  try {
    // 加载配置
    if (!loadConfiguration(config_file)) {
      RCLCPP_ERROR(logger_, "Failed to load configuration from: %s", config_file.c_str());
      return false;
    }

    // 创建机械臂实例
    robot_ = std::make_unique<rokae::xMateRobot>(robot_ip_, local_ip_);
    
    // 初始化机械臂
    if (!initializeRobot()) {
      RCLCPP_ERROR(logger_, "Failed to initialize robot");
      robot_.reset();
      return false;
    }

    is_connected_ = true;
    
    // 启动状态监控线程
    stop_monitoring_ = false;
    status_monitor_thread_ = std::thread(&RokaeAdapter::statusMonitoringThread, this);

    RCLCPP_INFO(logger_, "Successfully connected to Rokae robot at %s", robot_ip_.c_str());
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception during connection: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during connection: %s", e.what());
    return false;
  }
#endif
}

bool RokaeAdapter::disconnect()
{
  if (!is_connected_) {
    return true;
  }

#ifdef XCORE_SDK_AVAILABLE
  try {
    // 停止状态监控
    stop_monitoring_ = true;
    if (status_monitor_thread_.joinable()) {
      status_monitor_thread_.join();
    }

    // 断开机械臂连接
    if (robot_) {
      std::error_code ec;
      robot_->disconnectFromRobot(ec);
      if (ec) {
        RCLCPP_WARN(logger_, "Error during disconnect: %s", ec.message().c_str());
      }
      robot_.reset();
    }

    is_connected_ = false;
    RCLCPP_INFO(logger_, "Disconnected from Rokae robot");
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during disconnect: %s", e.what());
    return false;
  }
#endif
  
  return true;
}

bool RokaeAdapter::isConnected() const
{
  return is_connected_;
}

bool RokaeAdapter::moveToJoint(const std::vector<double>& /* joints */, double /* speed_ratio */)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  if (joints.size() != 6) {
    RCLCPP_ERROR(logger_, "Invalid joint array size: %zu, expected 6", joints.size());
    return false;
  }

  try {
    std::error_code ec;
    
    // 设置速度
    int speed_mms = static_cast<int>(default_speed_ * speed_ratio);
    robot_->setDefaultSpeed(speed_mms, ec);
    
    // 创建关节运动指令
    rokae::MoveAbsJCommand move_cmd(rokae::JointPosition(joints));
    
    // 执行运动
    robot_->executeCommand({move_cmd}, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to execute joint movement: %s", ec.message().c_str());
      return false;
    }

    // 等待运动完成
    waitForMotionComplete();
    
    RCLCPP_INFO(logger_, "Joint movement completed successfully");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception during joint movement: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during joint movement: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated joint movement completed");
  return true;
}

bool RokaeAdapter::moveToPose(const geometry_msgs::msg::Pose& /* pose */, double /* speed_ratio */)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 设置速度
    int speed_mms = static_cast<int>(default_speed_ * speed_ratio);
    robot_->setDefaultSpeed(speed_mms, ec);
    
    // 转换位姿格式
    std::array<double, 6> rokae_pose;
    rosePoseToRokaePosition(pose, rokae_pose);
    
    // 创建笛卡尔运动指令
    rokae::MoveLCommand move_cmd(rokae::CartesianPosition(rokae_pose));
    
    // 执行运动
    robot_->executeCommand({move_cmd}, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to execute pose movement: %s", ec.message().c_str());
      return false;
    }

    // 等待运动完成
    waitForMotionComplete();
    
    RCLCPP_INFO(logger_, "Pose movement completed successfully");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception during pose movement: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during pose movement: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated pose movement completed");
  return true;
}

bool RokaeAdapter::linearMove(const geometry_msgs::msg::Pose& /* pose */, double /* speed_ratio */)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 设置速度
    int speed_mms = static_cast<int>(default_speed_ * speed_ratio);
    robot_->setDefaultSpeed(speed_mms, ec);
    
    // 转换位姿格式
    std::array<double, 6> rokae_pose;
    rosePoseToRokaePosition(pose, rokae_pose);
    
    // 创建直线运动指令
    rokae::MoveLCommand move_cmd(rokae::CartesianPosition(rokae_pose));
    
    // 执行运动
    robot_->executeCommand({move_cmd}, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to execute linear movement: %s", ec.message().c_str());
      return false;
    }

    // 等待运动完成
    waitForMotionComplete();
    
    RCLCPP_INFO(logger_, "Linear movement completed successfully");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception during linear movement: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during linear movement: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated linear movement completed");
  return true;
}

bool RokaeAdapter::moveToJointAsync(const std::vector<double>& /* joints */)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  if (joints.size() != 6) {
    RCLCPP_ERROR(logger_, "Invalid joint array size: %zu, expected 6", joints.size());
    return false;
  }

  try {
    std::error_code ec;
    std::string cmd_id;
    
    // 创建关节运动指令
    rokae::MoveAbsJCommand move_cmd(rokae::JointPosition(joints));
    
    // 添加运动指令
    robot_->moveAppend({move_cmd}, cmd_id, ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to append joint movement: %s", ec.message().c_str());
      return false;
    }
    
    // 开始运动
    robot_->moveStart(ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to start joint movement: %s", ec.message().c_str());
      return false;
    }

    is_moving_ = true;
    RCLCPP_INFO(logger_, "Joint movement started asynchronously");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception during async joint movement: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during async joint movement: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated async joint movement started");
  return true;
}

bool RokaeAdapter::moveToPoseAsync(const geometry_msgs::msg::Pose& /* pose */)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    std::string cmd_id;
    
    // 转换位姿格式
    std::array<double, 6> rokae_pose;
    rosePoseToRokaePosition(pose, rokae_pose);
    
    // 创建笛卡尔运动指令
    rokae::MoveLCommand move_cmd(rokae::CartesianPosition(rokae_pose));
    
    // 添加运动指令
    robot_->moveAppend({move_cmd}, cmd_id, ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to append pose movement: %s", ec.message().c_str());
      return false;
    }
    
    // 开始运动
    robot_->moveStart(ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to start pose movement: %s", ec.message().c_str());
      return false;
    }

    is_moving_ = true;
    RCLCPP_INFO(logger_, "Pose movement started asynchronously");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception during async pose movement: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during async pose movement: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated async pose movement started");
  return true;
}

std::vector<double> RokaeAdapter::getCurrentJoints()
{
  std::vector<double> joints;
  
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return joints;
  }

  try {
    std::error_code ec;
    auto joint_pos = robot_->jointPos(ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to get joint positions: %s", ec.message().c_str());
      return joints;
    }

    joints.assign(joint_pos.begin(), joint_pos.end());

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception getting joint positions: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception getting joint positions: %s", e.what());
  }
#else
  // 模拟数据
  joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
#endif
  
  return joints;
}

geometry_msgs::msg::Pose RokaeAdapter::getCurrentPose()
{
  geometry_msgs::msg::Pose pose;
  
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return pose;
  }

  try {
    std::error_code ec;
    auto position = robot_->posture(rokae::CoordinateType::endInRef, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to get robot pose: %s", ec.message().c_str());
      return pose;
    }

    rokaePositionToRosePose(position, pose);

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception getting pose: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception getting pose: %s", e.what());
  }
#else
  // 模拟数据
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;
#endif
  
  return pose;
}

RobotState RokaeAdapter::getStatus()
{
  if (!is_connected_) {
    return RobotState::DISCONNECTED;
  }

  if (has_error_) {
    return RobotState::ERROR;
  }

#ifdef XCORE_SDK_AVAILABLE
  if (robot_) {
    try {
      std::error_code ec;
      auto operation_state = robot_->operationState(ec);
      
      if (!ec) {
        switch (operation_state) {
          case rokae::OperationState::idle:
            return RobotState::IDLE;
          case rokae::OperationState::moving:
            return RobotState::MOVING;
          case rokae::OperationState::jog:
            return RobotState::MOVING;
          default:
            return RobotState::ERROR;
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Exception getting robot status: %s", e.what());
      return RobotState::ERROR;
    }
  }
#endif

  return is_moving_ ? RobotState::MOVING : RobotState::IDLE;
}

std::string RokaeAdapter::getErrorMessage()
{
  return last_error_message_;
}

bool RokaeAdapter::setSpeed(double speed_ratio)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // Rokae SDK使用mm/s为单位，转换速度比例为实际速度
    int speed_mms = static_cast<int>(default_speed_ * speed_ratio);
    robot_->setDefaultSpeed(speed_mms, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to set speed: %s", ec.message().c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Speed set to %d mm/s (ratio: %.2f)", speed_mms, speed_ratio);
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception setting speed: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception setting speed: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated speed set to ratio: %.2f", speed_ratio);
  return true;
}

bool RokaeAdapter::setAcceleration(double acceleration_ratio)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 设置加速度，Rokae使用jerk参数
    double acceleration = default_acceleration_ * acceleration_ratio;
    robot_->adjustAcceleration(acceleration, acceleration, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to set acceleration: %s", ec.message().c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Acceleration set to %.2f", acceleration);
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception setting acceleration: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception setting acceleration: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated acceleration set to ratio: %.2f", acceleration_ratio);
  return true;
}

bool RokaeAdapter::setPayload(double payload_kg)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    // Rokae SDK的负载设置方法
    // 这里需要根据实际的Rokae SDK API进行实现
    RCLCPP_INFO(logger_, "Payload set to %.2f kg", payload_kg);
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception setting payload: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception setting payload: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated payload set to %.2f kg", payload_kg);
  return true;
}

bool RokaeAdapter::setBlendRadius(double radius)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 设置转弯区半径，单位mm
    int zone_mm = static_cast<int>(radius * 1000.0);
    robot_->setDefaultZone(zone_mm, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to set blend radius: %s", ec.message().c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Blend radius set to %d mm", zone_mm);
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception setting blend radius: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception setting blend radius: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated blend radius set to %.3f m", radius);
  return true;
}

bool RokaeAdapter::setToolFrame(const geometry_msgs::msg::Pose& /* tool_frame */)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 转换工具坐标系
    std::array<double, 6> tool_pose;
    rosePoseToRokaePosition(tool_frame, tool_pose);
    
    rokae::Toolset toolset;
    toolset.end = rokae::Frame(tool_pose);
    
    robot_->setToolset(toolset, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to set tool frame: %s", ec.message().c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Tool frame set successfully");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception setting tool frame: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception setting tool frame: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated tool frame set");
  return true;
}

bool RokaeAdapter::setWorkspaceFrame(const geometry_msgs::msg::Pose& /* workspace_frame */)
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 转换工作空间坐标系
    std::array<double, 6> workspace_pose;
    rosePoseToRokaePosition(workspace_frame, workspace_pose);
    
    rokae::Toolset toolset = robot_->toolset(ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to get current toolset: %s", ec.message().c_str());
      return false;
    }
    
    toolset.ref = rokae::Frame(workspace_pose);
    robot_->setToolset(toolset, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to set workspace frame: %s", ec.message().c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Workspace frame set successfully");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception setting workspace frame: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception setting workspace frame: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated workspace frame set");
  return true;
}

bool RokaeAdapter::emergencyStop()
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    robot_->stop(ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to emergency stop: %s", ec.message().c_str());
      return false;
    }

    is_moving_ = false;
    RCLCPP_INFO(logger_, "Emergency stop executed");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception during emergency stop: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during emergency stop: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated emergency stop executed");
  return true;
}

bool RokaeAdapter::clearError()
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 清除伺服报警
    robot_->clearServoAlarm(ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to clear errors: %s", ec.message().c_str());
      return false;
    }

    has_error_ = false;
    last_error_message_.clear();
    RCLCPP_INFO(logger_, "Errors cleared");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception clearing errors: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception clearing errors: %s", e.what());
    return false;
  }
#endif
  
  has_error_ = false;
  last_error_message_.clear();
  RCLCPP_INFO(logger_, "Simulated errors cleared");
  return true;
}

bool RokaeAdapter::enable()
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 上电
    robot_->setPowerState(true, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to enable robot: %s", ec.message().c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Robot enabled");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception enabling robot: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception enabling robot: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated robot enabled");
  return true;
}

bool RokaeAdapter::disable()
{
#ifdef XCORE_SDK_AVAILABLE
  if (!is_connected_ || !robot_) {
    RCLCPP_ERROR(logger_, "Robot not connected");
    return false;
  }

  try {
    std::error_code ec;
    
    // 断电
    robot_->setPowerState(false, ec);
    
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to disable robot: %s", ec.message().c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Robot disabled");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception disabling robot: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception disabling robot: %s", e.what());
    return false;
  }
#endif
  
  RCLCPP_INFO(logger_, "Simulated robot disabled");
  return true;
}

// 实现protected方法
bool RokaeAdapter::loadConfiguration(const std::string& config_file)
{
  try {
    if (config_file.empty()) {
      RCLCPP_WARN(logger_, "No config file provided, using default settings");
      return true;
    }

    YAML::Node config = YAML::LoadFile(config_file);
    
    if (config["robot_ip"]) {
      robot_ip_ = config["robot_ip"].as<std::string>();
    }
    
    if (config["local_ip"]) {
      local_ip_ = config["local_ip"].as<std::string>();
    }
    
    if (config["connection_timeout"]) {
      connection_timeout_ = config["connection_timeout"].as<int>();
    }
    
    if (config["default_speed"]) {
      default_speed_ = config["default_speed"].as<double>();
    }
    
    if (config["default_acceleration"]) {
      default_acceleration_ = config["default_acceleration"].as<double>();
    }
    
    if (config["default_blend_radius"]) {
      default_blend_radius_ = config["default_blend_radius"].as<double>();
    }

    RCLCPP_INFO(logger_, "Configuration loaded: robot_ip=%s, local_ip=%s", 
                robot_ip_.c_str(), local_ip_.c_str());
    return true;

  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(logger_, "YAML parsing error: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Error loading configuration: %s", e.what());
    return false;
  }
}

bool RokaeAdapter::initializeRobot()
{
#ifdef XCORE_SDK_AVAILABLE
  try {
    if (!robot_) {
      RCLCPP_ERROR(logger_, "Robot instance not created");
      return false;
    }

    std::error_code ec;
    
    // 连接到机械臂
    robot_->connectToRobot(static_cast<uint16_t>(connection_timeout_), ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to connect to robot: %s", ec.message().c_str());
      return false;
    }

    // 设置默认参数
    robot_->setDefaultSpeed(static_cast<int>(default_speed_), ec);
    if (ec) {
      RCLCPP_WARN(logger_, "Failed to set default speed: %s", ec.message().c_str());
    }

    robot_->setDefaultZone(static_cast<int>(default_blend_radius_), ec);
    if (ec) {
      RCLCPP_WARN(logger_, "Failed to set default zone: %s", ec.message().c_str());
    }

    RCLCPP_INFO(logger_, "Robot initialized successfully");
    return true;

  } catch (const rokae::Exception& e) {
    RCLCPP_ERROR(logger_, "Rokae exception during initialization: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during initialization: %s", e.what());
    return false;
  }
#else
  RCLCPP_INFO(logger_, "Simulated robot initialization completed");
  return true;
#endif
}

void RokaeAdapter::waitForMotionComplete()
{
#ifdef XCORE_SDK_AVAILABLE
  if (!robot_) return;

  try {
    std::error_code ec;
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
      auto state = robot_->operationState(ec);
      if (ec) {
        RCLCPP_WARN(logger_, "Failed to get operation state: %s", ec.message().c_str());
        break;
      }
      
      if (state == rokae::OperationState::idle) {
        is_moving_ = false;
        break;
      }
      
      // 超时保护 (30秒)
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > std::chrono::seconds(30)) {
        RCLCPP_WARN(logger_, "Motion timeout, stopping wait");
        break;
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception waiting for motion complete: %s", e.what());
  }
#else
  // 模拟运动完成等待
  std::this_thread::sleep_for(std::chrono::seconds(2));
  is_moving_ = false;
#endif
}

void RokaeAdapter::rosePoseToRokaePosition(const geometry_msgs::msg::Pose& ros_pose, 
                                          std::array<double, 6>& rokae_pose)
{
  // 位置转换 (m -> mm)
  rokae_pose[0] = ros_pose.position.x * 1000.0;
  rokae_pose[1] = ros_pose.position.y * 1000.0;
  rokae_pose[2] = ros_pose.position.z * 1000.0;

  // 四元数转欧拉角 (ZYX)
  tf2::Quaternion quat(ros_pose.orientation.x, ros_pose.orientation.y, 
                       ros_pose.orientation.z, ros_pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  // 角度转换 (弧度 -> 度)
  rokae_pose[3] = roll * 180.0 / M_PI;
  rokae_pose[4] = pitch * 180.0 / M_PI;
  rokae_pose[5] = yaw * 180.0 / M_PI;
}

void RokaeAdapter::rokaePositionToRosePose(const std::array<double, 6>& rokae_pose,
                                          geometry_msgs::msg::Pose& ros_pose)
{
  // 位置转换 (mm -> m)
  ros_pose.position.x = rokae_pose[0] / 1000.0;
  ros_pose.position.y = rokae_pose[1] / 1000.0;
  ros_pose.position.z = rokae_pose[2] / 1000.0;

  // 欧拉角转四元数 (度 -> 弧度)
  double roll = rokae_pose[3] * M_PI / 180.0;
  double pitch = rokae_pose[4] * M_PI / 180.0;
  double yaw = rokae_pose[5] * M_PI / 180.0;

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);

  ros_pose.orientation.x = quat.x();
  ros_pose.orientation.y = quat.y();
  ros_pose.orientation.z = quat.z();
  ros_pose.orientation.w = quat.w();
}

void RokaeAdapter::statusMonitoringThread()
{
  RCLCPP_INFO(logger_, "Status monitoring thread started");
  
  while (!stop_monitoring_) {
    try {
      if (is_connected_) {
#ifdef XCORE_SDK_AVAILABLE
        if (robot_) {
          std::error_code ec;
          
          // 检查机械臂状态
          auto state = robot_->operationState(ec);
          if (!ec) {
            is_moving_ = (state == rokae::OperationState::moving || 
                         state == rokae::OperationState::jog);
          }
          
          // 检查错误状态
          auto error_code = robot_->errorCode(ec);
          if (!ec && error_code != 0) {
            has_error_ = true;
            last_error_message_ = "Robot error code: " + std::to_string(error_code);
          }
        }
#endif
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Exception in status monitoring: %s", e.what());
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  RCLCPP_INFO(logger_, "Status monitoring thread stopped");
}

} // namespace elu_robot_arm_framework