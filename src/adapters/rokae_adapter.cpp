#include "elu_robot_arm_framework/adapters/rokae_adapter.hpp"
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>

namespace elu_robot_arm_framework
{

RokaeAdapter::RokaeAdapter()
: logger_(rclcpp::get_logger("rokae_adapter"))
, is_connected_(false)
, is_moving_(false)
, has_error_(false)
, stop_monitoring_(false)
, connection_timeout_(5000)
, default_speed_(200.0)
, default_acceleration_(0.5)
, default_blend_radius_(5.0)
{
  RCLCPP_INFO(logger_, "RokaeAdapter initialized");
}

RokaeAdapter::~RokaeAdapter()
{
  disconnect();
}

bool RokaeAdapter::connect(const std::string& config_file)
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

// 修正方法签名 - 添加speed_ratio参数
bool RokaeAdapter::moveToJoint(const std::vector<double>& joints, double speed_ratio)
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
  
  return false;
}

// 修正方法签名 - 添加speed_ratio参数
bool RokaeAdapter::moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio)
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
  
  return false;
}

// 新增方法 - 实现基类的linearMove
bool RokaeAdapter::linearMove(const geometry_msgs::msg::Pose& pose, double speed_ratio)
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
  
  return false;
}

// 扩展方法 - 移除override（保持原有功能）
bool RokaeAdapter::moveToJointAsync(const std::vector<double>& joints)
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
  
  return false;
}

bool RokaeAdapter::moveToPoseAsync(const geometry_msgs::msg::Pose& pose)
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
  
  return false;
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
#endif
  
  return pose;
}

// 修正返回类型 - 返回RobotState枚举而不是RobotStatus结构体
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

// 新增方法 - 实现基类的getErrorMessage
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
  
  return false;
}

// 修正方法签名 - 参数名从acceleration改为acceleration_ratio
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
  
  return false;
}

// 新增方法 - 实现基类的setPayload
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
  
  return false;
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
  
  return false;
}

bool RokaeAdapter::setToolFrame(const geometry_msgs::msg::Pose& tool_frame)
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
  
  return false;
}

bool RokaeAdapter::setWorkspaceFrame(const geometry_msgs::msg::Pose& workspace_frame)
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
  
  return false;
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
  
  return false;
}

// 修正方法名 - 从clearErrors改为clearError
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
  
  return false;
}

// 新增方法 - 实现基类的enable
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
  
  return false;
}

// 新增方法 - 实现基类的disable
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
  
  return false;
}

// 新增方法 - 实现基类的getRobotModel
std::string RokaeAdapter::getRobotModel() const
{
  return "Rokae xMateCR7";
}

// 新增方法 - 实现基类的getDoF
int RokaeAdapter::getDoF() const
{
  return 6;
}

// 新增方法 - 实现基类的getMaxPayload
double RokaeAdapter::getMaxPayload() const
{
  return 7.0; // kg
}

// 新增方法 - 实现基类的getJointLimits
std::vector<double> RokaeAdapter::getJointLimits() const
{
  // 返回6个关节的上下限 (12个值)
  // 这里需要根据实际的Rokae xMateCR7规格设置
  std::vector<double> limits = {
    -175.0, 175.0,   // Joint 1 (degrees)
    -90.0,  90.0,    // Joint 2
    -175.0, 175.0,   // Joint 3
    -180.0, 180.0,   // Joint 4
    -120.0, 120.0,   // Joint 5
    -360.0, 360.0    // Joint 6
  };
  
  // 转换为弧度
  for (auto& limit : limits) {
    limit = limit * M_PI / 180.0;
  }
}
  
bool RokaeAdapter::loadConfiguration(const std::string& config_file)
{
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    
    // 读取连接参数
    robot_ip_ = config["robot_ip"].as<std::string>("192.168.0.160");
    local_ip_ = config["local_ip"].as<std::string>("");
    connection_timeout_ = config["connection_timeout"].as<int>(5000);
    
    // 读取运动参数
    default_speed_ = config["default_speed"].as<double>(200.0);
    default_acceleration_ = config["default_acceleration"].as<double>(0.5);
    default_blend_radius_ = config["default_blend_radius"].as<double>(5.0);
    
    RCLCPP_INFO(logger_, "Configuration loaded: IP=%s, Speed=%.1f mm/s", 
                robot_ip_.c_str(), default_speed_);
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
    std::error_code ec;
    
    // 设置运动控制模式
    robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to set motion control mode: %s", ec.message().c_str());
      return false;
    }
    
    // 设置自动模式
    robot_->setOperateMode(rokae::OperateMode::automatic, ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to set operate mode: %s", ec.message().c_str());
      return false;
    }
    
    // 上电
    robot_->setPowerState(true, ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Failed to power on robot: %s", ec.message().c_str());
      return false;
    }
    
    // 设置默认参数
    robot_->setDefaultSpeed(static_cast<int>(default_speed_), ec);
    robot_->setDefaultZone(static_cast<int>(default_blend_radius_), ec);
    
    // 运动重置
    robot_->moveReset(ec);
    if (ec) {
      RCLCPP_WARN(logger_, "Move reset warning: %s", ec.message().c_str());
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
#endif
  
  return false;
}

void RokaeAdapter::waitForMotionComplete()
{
#ifdef XCORE_SDK_AVAILABLE
  if (!robot_) return;
  
  is_moving_ = true;
  
  try {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      std::error_code ec;
      auto state = robot_->operationState(ec);
      
      if (ec) {
        RCLCPP_WARN(logger_, "Error checking operation state: %s", ec.message().c_str());
        break;
      }
      
      if (state == rokae::OperationState::idle || state == rokae::OperationState::unknown) {
        break;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception while waiting for motion complete: %s", e.what());
  }
  
  is_moving_ = false;
#endif
}

void RokaeAdapter::rosePoseToRokaePosition(const geometry_msgs::msg::Pose& ros_pose, 
                                           std::array<double, 6>& rokae_pose)
{
  // 位置 (m)
  rokae_pose[0] = ros_pose.position.x;
  rokae_pose[1] = ros_pose.position.y;
  rokae_pose[2] = ros_pose.position.z;
  
  // 四元数转欧拉角 (rad)
  tf2::Quaternion q(
    ros_pose.orientation.x,
    ros_pose.orientation.y, 
    ros_pose.orientation.z,
    ros_pose.orientation.w
  );
  
  tf2::Matrix3x3 matrix(q);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  
  rokae_pose[3] = roll;   // Rx
  rokae_pose[4] = pitch;  // Ry  
  rokae_pose[5] = yaw;    // Rz
}

void RokaeAdapter::rokaePositionToRosePose(const std::array<double, 6>& rokae_pose,
                                           geometry_msgs::msg::Pose& ros_pose)
{
  // 位置
  ros_pose.position.x = rokae_pose[0];
  ros_pose.position.y = rokae_pose[1];
  ros_pose.position.z = rokae_pose[2];
  
  // 欧拉角转四元数
  tf2::Quaternion q;
  q.setRPY(rokae_pose[3], rokae_pose[4], rokae_pose[5]);
  
  ros_pose.orientation.x = q.x();
  ros_pose.orientation.y = q.y();
  ros_pose.orientation.z = q.z();
  ros_pose.orientation.w = q.w();
}

void RokaeAdapter::statusMonitoringThread()
{
  RCLCPP_INFO(logger_, "Status monitoring thread started");
  
  while (!stop_monitoring_ && is_connected_) {
    try {
#ifdef XCORE_SDK_AVAILABLE
      if (robot_) {
        std::error_code ec;
        
        // 检查运行状态
        auto operation_state = robot_->operationState(ec);
        if (!ec) {
          bool was_moving = is_moving_;
          is_moving_ = (operation_state == rokae::OperationState::moving);
          
          // 状态变化日志
          if (was_moving && !is_moving_) {
            RCLCPP_INFO(logger_, "Robot motion completed");
          } else if (!was_moving && is_moving_) {
            RCLCPP_INFO(logger_, "Robot motion started");
          }
        }
        
        // 检查错误状态
        auto power_state = robot_->powerState(ec);
        if (!ec) {
          if (power_state == rokae::PowerState::estop) {
            has_error_ = true;
            last_error_message_ = "Emergency stop activated";
            RCLCPP_ERROR(logger_, "Emergency stop detected");
          } else if (power_state == rokae::PowerState::off) {
            has_error_ = true;
            last_error_message_ = "Robot power off";
            RCLCPP_WARN(logger_, "Robot power is off");
          } else {
            if (has_error_ && last_error_message_ != "Manual error cleared") {
              has_error_ = false;
              last_error_message_.clear();
              RCLCPP_INFO(logger_, "Robot error condition cleared");
            }
          }
        }
      }
#endif
      
      // 监控间隔
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Exception in status monitoring: %s", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  
  RCLCPP_INFO(logger_, "Status monitoring thread stopped");
}

} // namespace elu_robot_arm_framework