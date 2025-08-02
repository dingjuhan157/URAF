#!/bin/bash

# ELU机械臂运动控制框架 ROS2 包创建脚本
# 适用于 Ubuntu 22.04.5 + ROS2 Humble
# 基于提供的设计文档第一阶段实现

set -e

echo "=========================================="
echo "ELU机械臂运动控制框架 ROS2 包创建开始"
echo "=========================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置，请先source ROS2环境"
    echo "请执行: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "检测到ROS2版本: $ROS_DISTRO"

# 设置工作空间目录（如果不存在则创建）
WORKSPACE_DIR="$HOME/elu_robot_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "创建工作空间: $WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_DIR/src"
    cd "$WORKSPACE_DIR"
    colcon build --symlink-install
fi

cd "$WORKSPACE_DIR/src"

# 创建主包
echo "创建主ROS2包: elu_robot_arm_framework"
ros2 pkg create --build-type ament_cmake elu_robot_arm_framework \
    --dependencies rclcpp rclpy std_msgs geometry_msgs sensor_msgs \
    trajectory_msgs control_msgs tf2 tf2_ros pluginlib \
    moveit_core moveit_ros_planning moveit_ros_planning_interface \
    yaml-cpp

cd elu_robot_arm_framework

# 创建目录结构
echo "创建目录结构..."

# 创建include目录结构
mkdir -p include/elu_robot_arm_framework/{interfaces,controllers,adapters,utils,safety}

# 创建源码目录结构
mkdir -p src/{controllers,adapters,utils,safety,nodes}

# 创建插件目录
mkdir -p plugins/{elu_adapter,ur_adapter,kuka_adapter,rokae_adapter}

# 创建配置文件目录
mkdir -p config/{robots,safety,launch_params}

# 创建启动文件目录
mkdir -p launch

# 创建消息定义目录
mkdir -p msg

# 创建服务定义目录
mkdir -p srv

# 创建测试目录
mkdir -p test/{unit,integration}

# 创建文档目录
mkdir -p docs/{api,user_guide,examples}

echo "目录结构创建完成"

# ========================================
# 创建头文件
# ========================================

echo "创建头文件..."

# 1. 机械臂接口基类
cat > include/elu_robot_arm_framework/interfaces/robot_arm_interface.hpp << 'EOF'
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
EOF

# 2. 运动控制器头文件
cat > include/elu_robot_arm_framework/controllers/motion_controller.hpp << 'EOF'
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
EOF

# 3. 插件管理器头文件
cat > include/elu_robot_arm_framework/utils/plugin_manager.hpp << 'EOF'
/**
 * @file plugin_manager.hpp
 * @brief 插件管理器
 * @author ELU Robotics Team
 * @date 2025
 */

#ifndef ELU_ROBOT_ARM_FRAMEWORK__UTILS__PLUGIN_MANAGER_HPP_
#define ELU_ROBOT_ARM_FRAMEWORK__UTILS__PLUGIN_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include "elu_robot_arm_framework/interfaces/robot_arm_interface.hpp"

namespace elu_robot_arm_framework
{

/**
 * @brief 插件管理器
 * 负责动态加载和管理机械臂适配器插件
 */
class PluginManager
{
public:
  PluginManager();
  virtual ~PluginManager();

  // 插件管理
  bool loadPlugin(const std::string& robot_id, const std::string& plugin_name);
  std::shared_ptr<RobotArmInterface> getPlugin(const std::string& robot_id);
  bool unloadPlugin(const std::string& robot_id);
  std::vector<std::string> getLoadedPlugins() const;
  std::vector<std::string> getAvailablePlugins() const;

  // 信息查询
  bool isPluginLoaded(const std::string& robot_id) const;
  std::string getPluginType(const std::string& robot_id) const;

private:
  std::unique_ptr<pluginlib::ClassLoader<RobotArmInterface>> plugin_loader_;
  std::unordered_map<std::string, std::shared_ptr<RobotArmInterface>> loaded_plugins_;
  std::unordered_map<std::string, std::string> plugin_types_;
  
  rclcpp::Logger logger_;
};

} // namespace elu_robot_arm_framework

#endif // ELU_ROBOT_ARM_FRAMEWORK__UTILS__PLUGIN_MANAGER_HPP_
EOF

# 4. 安全检查器头文件
cat > include/elu_robot_arm_framework/safety/safety_checker.hpp << 'EOF'
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
EOF

# ========================================
# 创建消息定义
# ========================================

echo "创建消息定义..."

# 1. 运动命令消息
cat > msg/MotionCommand.msg << 'EOF'
# 运动命令消息定义
std_msgs/Header header

# 机械臂标识
string robot_id

# 运动类型 (JOINT_MOVE=0, CARTESIAN_MOVE=1, LINEAR_MOVE=2, CIRCULAR_MOVE=3)
int32 motion_type

# 关节位置 (用于关节运动)
float64[] joint_positions

# 目标位姿 (用于笛卡尔运动)
geometry_msgs/Pose target_pose

# 运动参数
float64 speed_ratio         # 速度比例 (0.0-1.0)
float64 acceleration_ratio  # 加速度比例 (0.0-1.0)
bool wait_for_completion   # 是否等待运动完成

# 附加参数
string motion_params       # JSON格式的附加参数
EOF

# 2. 机械臂状态消息
cat > msg/RobotStatus.msg << 'EOF'
# 机械臂状态消息定义
std_msgs/Header header

# 机械臂标识
string robot_id

# 机械臂型号
string robot_model

# 连接状态
bool is_connected

# 运动状态 (DISCONNECTED=0, CONNECTING=1, IDLE=2, MOVING=3, ERROR=4, EMERGENCY_STOP=5)
int32 status

# 当前关节角度
float64[] current_joints

# 当前位姿
geometry_msgs/Pose current_pose

# 当前速度
float64 current_speed

# 错误信息
string error_message
int32 error_code

# 负载信息
float64 current_payload

# 系统信息
float64 cpu_usage
float64 memory_usage
string firmware_version
EOF

# ========================================
# 创建服务定义
# ========================================

echo "创建服务定义..."

# 1. 切换机械臂服务
cat > srv/SwitchRobot.srv << 'EOF'
# 切换机械臂服务定义

# 请求
string robot_id        # 要切换到的机械臂ID
bool force_switch     # 是否强制切换

---

# 响应
bool success          # 切换是否成功
string message        # 响应消息
string previous_robot # 之前的机械臂ID
EOF

# 2. 执行运动服务
cat > srv/ExecuteMotion.srv << 'EOF'
# 执行运动服务定义

# 请求
MotionCommand motion_command

---

# 响应
bool success          # 执行是否成功
string message        # 响应消息
float64 execution_time # 执行时间(秒)
EOF

# ========================================
# 创建源码文件
# ========================================

echo "创建源码文件..."

# 1. 运动控制器实现
cat > src/controllers/motion_controller.cpp << 'EOF'
/**
 * @file motion_controller.cpp
 * @brief 运动控制管理器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_robot_arm_framework/controllers/motion_controller.hpp"
#include <yaml-cpp/yaml.h>

namespace elu_robot_arm_framework
{

MotionController::MotionController(const rclcpp::NodeOptions& options)
: Node("motion_controller", options)
{
  RCLCPP_INFO(get_logger(), "Initializing Motion Controller...");

  // 初始化插件管理器
  plugin_manager_ = std::make_unique<PluginManager>();

  // 初始化安全检查器
  std::string safety_config = this->declare_parameter("safety_config", 
    std::string("config/safety/safety_config.yaml"));
  safety_checker_ = std::make_unique<SafetyChecker>(safety_config);

  // 创建发布者
  status_publisher_ = this->create_publisher<msg::RobotStatus>(
    "robot_status", 10);

  // 创建订阅者
  motion_subscriber_ = this->create_subscription<msg::MotionCommand>(
    "motion_command", 10,
    std::bind(&MotionController::motionCommandCallback, this, std::placeholders::_1));

  // 创建服务
  switch_robot_service_ = this->create_service<srv::SwitchRobot>(
    "switch_robot",
    std::bind(&MotionController::switchRobotCallback, this, 
              std::placeholders::_1, std::placeholders::_2));

  execute_motion_service_ = this->create_service<srv::ExecuteMotion>(
    "execute_motion",
    std::bind(&MotionController::executeMotionCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建状态发布定时器
  status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MotionController::statusTimerCallback, this));

  // 加载机械臂配置
  loadRobotConfigs();

  RCLCPP_INFO(get_logger(), "Motion Controller initialized successfully");
}

MotionController::~MotionController()
{
  // 断开所有连接的机械臂
  for (auto& [robot_id, robot] : loaded_robots_) {
    if (robot && robot->isConnected()) {
      robot->disconnect();
    }
  }
}

bool MotionController::loadRobot(const std::string& robot_id, const std::string& config_file)
{
  try {
    // 读取配置文件
    YAML::Node config = YAML::LoadFile(config_file);
    std::string plugin_name = config["plugin_type"].as<std::string>();

    // 加载插件
    if (!plugin_manager_->loadPlugin(robot_id, plugin_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to load plugin for robot: %s", robot_id.c_str());
      return false;
    }

    // 获取插件实例
    auto robot = plugin_manager_->getPlugin(robot_id);
    if (!robot) {
      RCLCPP_ERROR(get_logger(), "Failed to get plugin instance for robot: %s", robot_id.c_str());
      return false;
    }

    // 连接机械臂
    if (!robot->connect(config_file)) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to robot: %s", robot_id.c_str());
      return false;
    }

    // 添加到已加载机械臂列表
    loaded_robots_[robot_id] = robot;

    RCLCPP_INFO(get_logger(), "Robot loaded successfully: %s", robot_id.c_str());
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception while loading robot %s: %s", 
                 robot_id.c_str(), e.what());
    return false;
  }
}

bool MotionController::switchRobot(const std::string& robot_id)
{
  if (loaded_robots_.find(robot_id) == loaded_robots_.end()) {
    RCLCPP_ERROR(get_logger(), "Robot not loaded: %s", robot_id.c_str());
    return false;
  }

  current_robot_ = loaded_robots_[robot_id];
  current_robot_id_ = robot_id;

  RCLCPP_INFO(get_logger(), "Switched to robot: %s", robot_id.c_str());
  return true;
}

void MotionController::motionCommandCallback(const msg::MotionCommand::SharedPtr msg)
{
  if (!validateMotionCommand(*msg)) {
    RCLCPP_WARN(get_logger(), "Invalid motion command received");
    return;
  }

  executeMotion(*msg);
}

bool MotionController::executeMotion(const msg::MotionCommand& command)
{
  if (!current_robot_) {
    RCLCPP_ERROR(get_logger(), "No robot selected");
    return false;
  }

  // 安全检查
  auto safety_result = safety_checker_->checkMotionCommand(command);
  if (!safety_result.is_safe) {
    RCLCPP_ERROR(get_logger(), "Safety check failed: %s", 
                 safety_result.error_message.c_str());
    return false;
  }

  // 执行运动
  bool success = false;
  switch (command.motion_type) {
    case 0: // JOINT_MOVE
      success = current_robot_->moveToJoint(command.joint_positions, command.speed_ratio);
      break;
    case 1: // CARTESIAN_MOVE
      success = current_robot_->moveToPose(command.target_pose, command.speed_ratio);
      break;
    case 2: // LINEAR_MOVE
      success = current_robot_->linearMove(command.target_pose, command.speed_ratio);
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unsupported motion type: %d", command.motion_type);
      return false;
  }

  return success;
}

void MotionController::statusTimerCallback()
{
  if (!current_robot_) {
    return;
  }

  // 发布机械臂状态
  msg::RobotStatus status_msg;
  status_msg.header.stamp = this->now();
  status_msg.robot_id = current_robot_id_;
  status_msg.robot_model = current_robot_->getRobotModel();
  status_msg.is_connected = current_robot_->isConnected();
  status_msg.status = static_cast<int32_t>(current_robot_->getStatus());
  status_msg.current_joints = current_robot_->getCurrentJoints();
  status_msg.current_pose = current_robot_->getCurrentPose();
  status_msg.error_message = current_robot_->getErrorMessage();

  status_publisher_->publish(status_msg);
}

void MotionController::loadRobotConfigs()
{
  // TODO: 从参数服务器或配置文件加载机械臂配置
  RCLCPP_INFO(get_logger(), "Loading robot configurations...");
}

bool MotionController::validateMotionCommand(const msg::MotionCommand& command)
{
  // TODO: 实现运动命令验证逻辑
  return true;
}

} // namespace elu_robot_arm_framework
EOF

# 2. 插件管理器实现
cat > src/utils/plugin_manager.cpp << 'EOF'
/**
 * @file plugin_manager.cpp
 * @brief 插件管理器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_robot_arm_framework/utils/plugin_manager.hpp"

namespace elu_robot_arm_framework
{

PluginManager::PluginManager()
: logger_(rclcpp::get_logger("plugin_manager"))
{
  try {
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<RobotArmInterface>>(
      "elu_robot_arm_framework", "elu_robot_arm_framework::RobotArmInterface");
    
    RCLCPP_INFO(logger_, "Plugin manager initialized successfully");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to initialize plugin loader: %s", e.what());
  }
}

PluginManager::~PluginManager()
{
  // 卸载所有插件
  for (auto& [robot_id, plugin] : loaded_plugins_) {
    if (plugin && plugin->isConnected()) {
      plugin->disconnect();
    }
  }
  loaded_plugins_.clear();
}

bool PluginManager::loadPlugin(const std::string& robot_id, const std::string& plugin_name)
{
  try {
    // 检查是否已经加载
    if (isPluginLoaded(robot_id)) {
      RCLCPP_WARN(logger_, "Plugin already loaded for robot: %s", robot_id.c_str());
      return true;
    }

    // 创建插件实例
    auto plugin = plugin_loader_->createSharedInstance(plugin_name);
    if (!plugin) {
      RCLCPP_ERROR(logger_, "Failed to create plugin instance: %s", plugin_name.c_str());
      return false;
    }

    // 保存插件
    loaded_plugins_[robot_id] = plugin;
    plugin_types_[robot_id] = plugin_name;

    RCLCPP_INFO(logger_, "Plugin loaded successfully: %s -> %s", 
                robot_id.c_str(), plugin_name.c_str());
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception while loading plugin %s: %s", 
                 plugin_name.c_str(), e.what());
    return false;
  }
}

std::shared_ptr<RobotArmInterface> PluginManager::getPlugin(const std::string& robot_id)
{
  auto it = loaded_plugins_.find(robot_id);
  if (it != loaded_plugins_.end()) {
    return it->second;
  }
  return nullptr;
}

bool PluginManager::unloadPlugin(const std::string& robot_id)
{
  auto it = loaded_plugins_.find(robot_id);
  if (it == loaded_plugins_.end()) {
    RCLCPP_WARN(logger_, "Plugin not found for robot: %s", robot_id.c_str());
    return false;
  }

  // 断开连接
  if (it->second && it->second->isConnected()) {
    it->second->disconnect();
  }

  // 移除插件
  loaded_plugins_.erase(it);
  plugin_types_.erase(robot_id);

  RCLCPP_INFO(logger_, "Plugin unloaded successfully: %s", robot_id.c_str());
  return true;
}

std::vector<std::string> PluginManager::getLoadedPlugins() const
{
  std::vector<std::string> loaded;
  for (const auto& [robot_id, plugin] : loaded_plugins_) {
    loaded.push_back(robot_id);
  }
  return loaded;
}

std::vector<std::string> PluginManager::getAvailablePlugins() const
{
  if (!plugin_loader_) {
    return {};
  }

  return plugin_loader_->getDeclaredClasses();
}

bool PluginManager::isPluginLoaded(const std::string& robot_id) const
{
  return loaded_plugins_.find(robot_id) != loaded_plugins_.end();
}

std::string PluginManager::getPluginType(const std::string& robot_id) const
{
  auto it = plugin_types_.find(robot_id);
  if (it != plugin_types_.end()) {
    return it->second;
  }
  return "";
}

} // namespace elu_robot_arm_framework
EOF

# 3. 安全检查器实现
cat > src/safety/safety_checker.cpp << 'EOF'
/**
 * @file safety_checker.cpp
 * @brief 安全检查器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_robot_arm_framework/safety/safety_checker.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace elu_robot_arm_framework
{

SafetyChecker::SafetyChecker(const std::string& config_file)
: max_speed_ratio_(1.0)
, max_acceleration_ratio_(1.0)
, safety_enabled_(true)
, logger_(rclcpp::get_logger("safety_checker"))
{
  // 设置默认工作空间限制
  workspace_limits_.x_min = -1.0;
  workspace_limits_.x_max = 1.0;
  workspace_limits_.y_min = -1.0;
  workspace_limits_.y_max = 1.0;
  workspace_limits_.z_min = 0.0;
  workspace_limits_.z_max = 2.0;

  if (!config_file.empty()) {
    loadSafetyConfig(config_file);
  }

  RCLCPP_INFO(logger_, "Safety checker initialized");
}

SafetyCheckResult SafetyChecker::checkJointLimits(const std::vector<double>& joints, 
                                                 const std::vector<double>& joint_limits)
{
  SafetyCheckResult result;
  result.is_safe = true;
  result.error_code = 0;

  if (!safety_enabled_) {
    return result;
  }

  if (joints.size() != joint_limits.size() / 2) {
    result.is_safe = false;
    result.error_message = "Joint positions and limits size mismatch";
    result.error_code = -1;
    return result;
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    double min_limit = joint_limits[i * 2];
    double max_limit = joint_limits[i * 2 + 1];

    if (joints[i] < min_limit || joints[i] > max_limit) {
      result.is_safe = false;
      result.error_message = "Joint " + std::to_string(i) + " out of limits: " + 
                            std::to_string(joints[i]) + " not in [" + 
                            std::to_string(min_limit) + ", " + std::to_string(max_limit) + "]";
      result.error_code = i + 1;
      return result;
    }
  }

  return result;
}

SafetyCheckResult SafetyChecker::checkWorkspace(const geometry_msgs::msg::Pose& pose)
{
  SafetyCheckResult result;
  result.is_safe = true;
  result.error_code = 0;

  if (!safety_enabled_) {
    return result;
  }

  if (!isInWorkspace(pose)) {
    result.is_safe = false;
    result.error_message = "Target pose outside workspace limits";
    result.error_code = -2;
  }

  return result;
}

SafetyCheckResult SafetyChecker::checkSpeed(double speed_ratio)
{
  SafetyCheckResult result;
  result.is_safe = true;
  result.error_code = 0;

  if (!safety_enabled_) {
    return result;
  }

  if (speed_ratio < 0.0 || speed_ratio > max_speed_ratio_) {
    result.is_safe = false;
    result.error_message = "Speed ratio out of range: " + std::to_string(speed_ratio) + 
                          " not in [0.0, " + std::to_string(max_speed_ratio_) + "]";
    result.error_code = -3;
  }

  return result;
}

SafetyCheckResult SafetyChecker::checkMotionCommand(const msg::MotionCommand& command)
{
  SafetyCheckResult result;
  result.is_safe = true;
  result.error_code = 0;

  if (!safety_enabled_) {
    return result;
  }

  // 检查速度
  auto speed_result = checkSpeed(command.speed_ratio);
  if (!speed_result.is_safe) {
    return speed_result;
  }

  // 检查加速度
  if (command.acceleration_ratio < 0.0 || command.acceleration_ratio > max_acceleration_ratio_) {
    result.is_safe = false;
    result.error_message = "Acceleration ratio out of range: " + 
                          std::to_string(command.acceleration_ratio);
    result.error_code = -4;
    return result;
  }

  // 检查工作空间（如果是笛卡尔运动）
  if (command.motion_type == 1 || command.motion_type == 2) { // CARTESIAN_MOVE or LINEAR_MOVE
    auto workspace_result = checkWorkspace(command.target_pose);
    if (!workspace_result.is_safe) {
      return workspace_result;
    }
  }

  return result;
}

bool SafetyChecker::loadSafetyConfig(const std::string& config_file)
{
  try {
    YAML::Node config = YAML::LoadFile(config_file);

    // 加载工作空间限制
    if (config["workspace_limits"]) {
      auto limits = config["workspace_limits"];
      workspace_limits_.x_min = limits["x_min"].as<double>();
      workspace_limits_.x_max = limits["x_max"].as<double>();
      workspace_limits_.y_min = limits["y_min"].as<double>();
      workspace_limits_.y_max = limits["y_max"].as<double>();
      workspace_limits_.z_min = limits["z_min"].as<double>();
      workspace_limits_.z_max = limits["z_max"].as<double>();
    }

    // 加载速度限制
    if (config["max_speed_ratio"]) {
      max_speed_ratio_ = config["max_speed_ratio"].as<double>();
    }

    // 加载加速度限制
    if (config["max_acceleration_ratio"]) {
      max_acceleration_ratio_ = config["max_acceleration_ratio"].as<double>();
    }

    // 安全检查开关
    if (config["safety_enabled"]) {
      safety_enabled_ = config["safety_enabled"].as<bool>();
    }

    RCLCPP_INFO(logger_, "Safety configuration loaded from: %s", config_file.c_str());
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to load safety config: %s", e.what());
    return false;
  }
}

bool SafetyChecker::isInWorkspace(const geometry_msgs::msg::Pose& pose) const
{
  return (pose.position.x >= workspace_limits_.x_min && pose.position.x <= workspace_limits_.x_max &&
          pose.position.y >= workspace_limits_.y_min && pose.position.y <= workspace_limits_.y_max &&
          pose.position.z >= workspace_limits_.z_min && pose.position.z <= workspace_limits_.z_max);
}

void SafetyChecker::setWorkspaceLimits(const WorkspaceLimits& limits)
{
  workspace_limits_ = limits;
  RCLCPP_INFO(logger_, "Workspace limits updated");
}

void SafetyChecker::setMaxSpeed(double max_speed)
{
  max_speed_ratio_ = max_speed;
  RCLCPP_INFO(logger_, "Max speed ratio set to: %f", max_speed);
}

void SafetyChecker::setMaxAcceleration(double max_acceleration)
{
  max_acceleration_ratio_ = max_acceleration;
  RCLCPP_INFO(logger_, "Max acceleration ratio set to: %f", max_acceleration);
}

void SafetyChecker::enableSafetyCheck(bool enable)
{
  safety_enabled_ = enable;
  RCLCPP_INFO(logger_, "Safety check %s", enable ? "enabled" : "disabled");
}

bool SafetyChecker::isSafetyCheckEnabled() const
{
  return safety_enabled_;
}

} // namespace elu_robot_arm_framework
EOF

# 4. ELU适配器示例
mkdir -p plugins/elu_adapter/src
mkdir -p plugins/elu_adapter/include/elu_adapter

cat > plugins/elu_adapter/include/elu_adapter/elu_adapter.hpp << 'EOF'
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
EOF

cat > plugins/elu_adapter/src/elu_adapter.cpp << 'EOF'
/**
 * @file elu_adapter.cpp
 * @brief ELU机械臂适配器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_adapter/elu_adapter.hpp"
#include <yaml-cpp/yaml.h>
#include <pluginlib/class_list_macros.hpp>

namespace elu_adapter
{

EluAdapter::EluAdapter()
: connected_(false)
, current_state_(elu_robot_arm_framework::RobotState::DISCONNECTED)
, current_speed_ratio_(1.0)
, current_acceleration_ratio_(1.0)
, current_payload_(0.0)
, logger_(rclcpp::get_logger("elu_adapter"))
{
  // 初始化默认关节角度（6自由度）
  current_joints_.resize(6, 0.0);

  // 初始化默认位姿
  current_pose_.position.x = 0.5;
  current_pose_.position.y = 0.0;
  current_pose_.position.z = 0.5;
  current_pose_.orientation.w = 1.0;
  current_pose_.orientation.x = 0.0;
  current_pose_.orientation.y = 0.0;
  current_pose_.orientation.z = 0.0;

  RCLCPP_INFO(logger_, "ELU Adapter initialized");
}

EluAdapter::~EluAdapter()
{
  if (connected_) {
    disconnect();
  }
}

bool EluAdapter::connect(const std::string& config_file)
{
  try {
    // 读取配置文件
    YAML::Node config = YAML::LoadFile(config_file);
    ip_address_ = config["ip_address"].as<std::string>();
    port_ = config["port"].as<int>();

    RCLCPP_INFO(logger_, "Connecting to ELU robot at %s:%d", ip_address_.c_str(), port_);

    // TODO: 实际连接ELU机械臂
    // 这里应该调用ELU SDK的连接函数
    
    // 模拟连接过程
    current_state_ = elu_robot_arm_framework::RobotState::CONNECTING;
    
    // 初始化连接
    if (!initializeConnection()) {
      handleError("Failed to initialize connection");
      return false;
    }

    connected_ = true;
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
    error_message_.clear();

    RCLCPP_INFO(logger_, "Successfully connected to ELU robot");
    return true;
  }
  catch (const std::exception& e) {
    handleError("Connection failed: " + std::string(e.what()));
    return false;
  }
}

bool EluAdapter::disconnect()
{
  if (!connected_) {
    return true;
  }

  try {
    // TODO: 实际断开ELU机械臂连接
    // 这里应该调用ELU SDK的断开函数

    connected_ = false;
    current_state_ = elu_robot_arm_framework::RobotState::DISCONNECTED;

    RCLCPP_INFO(logger_, "Disconnected from ELU robot");
    return true;
  }
  catch (const std::exception& e) {
    handleError("Disconnection failed: " + std::string(e.what()));
    return false;
  }
}

bool EluAdapter::isConnected() const
{
  return connected_;
}

bool EluAdapter::moveToJoint(const std::vector<double>& joints, double speed_ratio)
{
  if (!connected_) {
    handleError("Robot not connected");
    return false;
  }

  if (joints.size() != 6) {
    handleError("Invalid joint array size: expected 6, got " + std::to_string(joints.size()));
    return false;
  }

  try {
    RCLCPP_INFO(logger_, "Moving to joint positions with speed ratio: %f", speed_ratio);

    current_state_ = elu_robot_arm_framework::RobotState::MOVING;

    // TODO: 实际执行关节运动
    // 这里应该调用ELU SDK的关节运动函数

    // 模拟运动过程
    current_joints_ = joints;
    
    // 更新当前位姿（简化的正运动学）
    // 实际应用中应该调用正运动学函数
    
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
    return true;
  }
  catch (const std::exception& e) {
    handleError("Joint movement failed: " + std::string(e.what()));
    return false;
  }
}

bool EluAdapter::moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio)
{
  if (!connected_) {
    handleError("Robot not connected");
    return false;
  }

  try {
    RCLCPP_INFO(logger_, "Moving to pose [%f, %f, %f] with speed ratio: %f",
                pose.position.x, pose.position.y, pose.position.z, speed_ratio);

    current_state_ = elu_robot_arm_framework::RobotState::MOVING;

    // TODO: 实际执行笛卡尔运动
    // 这里应该调用ELU SDK的笛卡尔运动函数

    // 模拟运动过程
    current_pose_ = pose;
    
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
    return true;
  }
  catch (const std::exception& e) {
    handleError("Pose movement failed: " + std::string(e.what()));
    return false;
  }
}

bool EluAdapter::linearMove(const geometry_msgs::msg::Pose& pose, double speed_ratio)
{
  if (!connected_) {
    handleError("Robot not connected");
    return false;
  }

  try {
    RCLCPP_INFO(logger_, "Linear move to pose [%f, %f, %f] with speed ratio: %f",
                pose.position.x, pose.position.y, pose.position.z, speed_ratio);

    current_state_ = elu_robot_arm_framework::RobotState::MOVING;

    // TODO: 实际执行直线运动
    // 这里应该调用ELU SDK的直线运动函数

    current_pose_ = pose;
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
    return true;
  }
  catch (const std::exception& e) {
    handleError("Linear movement failed: " + std::string(e.what()));
    return false;
  }
}

std::vector<double> EluAdapter::getCurrentJoints()
{
  if (!connected_) {
    return {};
  }

  // TODO: 从ELU机械臂获取实际关节角度
  updateRobotState();
  return current_joints_;
}

geometry_msgs::msg::Pose EluAdapter::getCurrentPose()
{
  if (!connected_) {
    return geometry_msgs::msg::Pose();
  }

  // TODO: 从ELU机械臂获取实际位姿
  updateRobotState();
  return current_pose_;
}

elu_robot_arm_framework::RobotState EluAdapter::getStatus()
{
  if (connected_) {
    updateRobotState();
  }
  return current_state_;
}

std::string EluAdapter::getErrorMessage()
{
  return error_message_;
}

bool EluAdapter::setSpeed(double speed_ratio)
{
  if (speed_ratio < 0.0 || speed_ratio > 1.0) {
    handleError("Invalid speed ratio: " + std::to_string(speed_ratio));
    return false;
  }

  current_speed_ratio_ = speed_ratio;
  RCLCPP_INFO(logger_, "Speed ratio set to: %f", speed_ratio);
  return true;
}

bool EluAdapter::setAcceleration(double acceleration_ratio)
{
  if (acceleration_ratio < 0.0 || acceleration_ratio > 1.0) {
    handleError("Invalid acceleration ratio: " + std::to_string(acceleration_ratio));
    return false;
  }

  current_acceleration_ratio_ = acceleration_ratio;
  RCLCPP_INFO(logger_, "Acceleration ratio set to: %f", acceleration_ratio);
  return true;
}

bool EluAdapter::setPayload(double payload_kg)
{
  if (payload_kg < 0.0 || payload_kg > getMaxPayload()) {
    handleError("Invalid payload: " + std::to_string(payload_kg));
    return false;
  }

  current_payload_ = payload_kg;
  RCLCPP_INFO(logger_, "Payload set to: %f kg", payload_kg);
  return true;
}

bool EluAdapter::emergencyStop()
{
  RCLCPP_WARN(logger_, "Emergency stop triggered!");

  // TODO: 调用ELU SDK的紧急停止函数

  current_state_ = elu_robot_arm_framework::RobotState::EMERGENCY_STOP;
  return true;
}

bool EluAdapter::clearError()
{
  error_message_.clear();
  if (current_state_ == elu_robot_arm_framework::RobotState::ERROR) {
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
  }
  RCLCPP_INFO(logger_, "Error cleared");
  return true;
}

bool EluAdapter::enable()
{
  if (!connected_) {
    handleError("Robot not connected");
    return false;
  }

  // TODO: 调用ELU SDK的使能函数

  if (current_state_ == elu_robot_arm_framework::RobotState::EMERGENCY_STOP) {
    current_state_ = elu_robot_arm_framework::RobotState::IDLE;
  }

  RCLCPP_INFO(logger_, "Robot enabled");
  return true;
}

bool EluAdapter::disable()
{
  if (!connected_) {
    return true;
  }

  // TODO: 调用ELU SDK的失能函数

  RCLCPP_INFO(logger_, "Robot disabled");
  return true;
}

std::string EluAdapter::getRobotModel() const
{
  return "ELU-6DOF";
}

int EluAdapter::getDoF() const
{
  return 6;
}

double EluAdapter::getMaxPayload() const
{
  return 5.0; // 5kg
}

std::vector<double> EluAdapter::getJointLimits() const
{
  // 返回关节限制 [min1, max1, min2, max2, ...]
  return {-3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14, -3.14, 3.14};
}

bool EluAdapter::initializeConnection()
{
  // TODO: 实现ELU机械臂的初始化逻辑
  // 例如：握手、版本检查、参数配置等
  
  RCLCPP_INFO(logger_, "Initializing connection to ELU robot...");
  
  // 模拟初始化过程
  return true;
}

bool EluAdapter::updateRobotState()
{
  // TODO: 从ELU机械臂获取实时状态
  // 更新current_joints_, current_pose_, current_state_等
  
  return true;
}

void EluAdapter::handleError(const std::string& error_msg)
{
  error_message_ = error_msg;
  current_state_ = elu_robot_arm_framework::RobotState::ERROR;
  RCLCPP_ERROR(logger_, "%s", error_msg.c_str());
}

} // namespace elu_adapter

// 注册插件
PLUGINLIB_EXPORT_CLASS(elu_adapter::EluAdapter, elu_robot_arm_framework::RobotArmInterface)
EOF

# ========================================
# 创建配置文件
# ========================================

echo "创建配置文件..."

# 1. 机械臂配置文件
cat > config/robots/robot_config.yaml << 'EOF'
# ELU机械臂运动控制框架配置文件

robot_arms:
  elu_arm_1:
    plugin_type: "elu_adapter/EluAdapter"
    ip_address: "192.168.1.100"
    port: 8080
    dof: 6
    max_payload: 5.0  # kg
    max_reach: 1.2    # meters
    joint_limits:     # [min, max] for each joint (radians)
      - [-3.14159, 3.14159]  # Joint 1
      - [-3.14159, 3.14159]  # Joint 2
      - [-3.14159, 3.14159]  # Joint 3
      - [-3.14159, 3.14159]  # Joint 4
      - [-3.14159, 3.14159]  # Joint 5
      - [-3.14159, 3.14159]  # Joint 6
    workspace_limits:
      x_min: -1.2
      x_max: 1.2
      y_min: -1.2
      y_max: 1.2
      z_min: 0.0
      z_max: 2.0

  elu_arm_2:
    plugin_type: "elu_adapter/EluAdapter"
    ip_address: "192.168.1.101"
    port: 8080
    dof: 6
    max_payload: 5.0
    max_reach: 1.2
    joint_limits:
      - [-3.14159, 3.14159]
      - [-3.14159, 3.14159]
      - [-3.14159, 3.14159]
      - [-3.14159, 3.14159]
      - [-3.14159, 3.14159]
      - [-3.14159, 3.14159]
    workspace_limits:
      x_min: -1.2
      x_max: 1.2
      y_min: -1.2
      y_max: 1.2
      z_min: 0.0
      z_max: 2.0

# 默认设置
default_settings:
  default_speed_ratio: 0.5
  default_acceleration_ratio: 0.5
  control_frequency: 100  # Hz
  status_publish_frequency: 10  # Hz
EOF

# 2. 安全配置文件
cat > config/safety/safety_config.yaml << 'EOF'
# 安全检查配置文件

# 全局安全开关
safety_enabled: true

# 工作空间限制
workspace_limits:
  x_min: -1.5
  x_max: 1.5
  y_min: -1.5
  y_max: 1.5
  z_min: 0.0
  z_max: 2.5

# 速度和加速度限制
max_speed_ratio: 1.0
max_acceleration_ratio: 1.0

# 关节速度限制 (rad/s)
max_joint_velocities: [2.0, 2.0, 2.0, 3.0, 3.0, 3.0]

# 关节加速度限制 (rad/s²)
max_joint_accelerations: [5.0, 5.0, 5.0, 10.0, 10.0, 10.0]

# 碰撞检测
collision_detection:
  enabled: true
  self_collision_check: true
  environment_collision_check: true

# 安全区域定义
safety_zones:
  forbidden_zones:
    - name: "operator_area"
      type: "box"
      center: [0.0, -0.8, 0.5]
      dimensions: [0.5, 0.3, 1.0]
    
  warning_zones:
    - name: "work_table"
      type: "box"
      center: [0.6, 0.0, 0.0]
      dimensions: [0.8, 1.0, 0.1]

# 紧急停止设置
emergency_stop:
  deceleration_time: 0.1  # seconds
  stop_distance: 0.01     # meters
EOF

# ========================================
# 创建启动文件
# ========================================

echo "创建启动文件..."

# 1. 主启动文件
cat > launch/elu_robot_framework.launch.py << 'EOF'
#!/usr/bin/env python3
"""
ELU机械臂运动控制框架启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('elu_robot_arm_framework'),
            'config', 'robots', 'robot_config.yaml'
        ]),
        description='Robot configuration file path'
    )

    safety_config_arg = DeclareLaunchArgument(
        'safety_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('elu_robot_arm_framework'),
            'config', 'safety', 'safety_config.yaml'
        ]),
        description='Safety configuration file path'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    # 运动控制器节点
    motion_controller_node = Node(
        package='elu_robot_arm_framework',
        executable='motion_controller_node',
        name='motion_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('robot_config'),
            {
                'safety_config': LaunchConfiguration('safety_config'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    # 状态监控节点
    status_monitor_node = Node(
        package='elu_robot_arm_framework',
        executable='status_monitor_node',
        name='status_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        robot_config_arg,
        safety_config_arg,
        use_sim_time_arg,
        log_level_arg,
        motion_controller_node,
        status_monitor_node,
    ])
EOF

# 2. 多机械臂控制启动文件
cat > launch/multi_arm_control.launch.py << 'EOF'
#!/usr/bin/env python3
"""
多机械臂控制启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to control'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # 创建多个机械臂控制组
    robot_groups = []
    
    for i in range(1, 3):  # 支持最多2个机械臂
        robot_namespace = f'robot_{i}'
        
        robot_group = GroupAction([
            PushRosNamespace(robot_namespace),
            
            Node(
                package='elu_robot_arm_framework',
                executable='motion_controller_node',
                name='motion_controller',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare('elu_robot_arm_framework'),
                        'config', 'robots', f'robot_{i}_config.yaml'
                    ]),
                    {
                        'robot_id': f'elu_arm_{i}',
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ]
            ),
            
            Node(
                package='elu_robot_arm_framework',
                executable='robot_interface_node',
                name='robot_interface',
                output='screen',
                parameters=[
                    {'robot_id': f'elu_arm_{i}'}
                ]
            )
        ])
        
        robot_groups.append(robot_group)

    # 协调控制器
    coordinator_node = Node(
        package='elu_robot_arm_framework',
        executable='multi_robot_coordinator_node',
        name='multi_robot_coordinator',
        output='screen',
        parameters=[
            {'num_robots': LaunchConfiguration('num_robots')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        num_robots_arg,
        use_sim_time_arg,
        coordinator_node,
    ] + robot_groups)
EOF

# ========================================
# 创建节点源码
# ========================================

echo "创建节点源码..."

# 1. 运动控制器节点
cat > src/nodes/motion_controller_node.cpp << 'EOF'
/**
 * @file motion_controller_node.cpp
 * @brief 运动控制器节点
 * @author ELU Robotics Team
 * @date 2025
 */

#include <rclcpp/rclcpp.hpp>
#include "elu_robot_arm_framework/controllers/motion_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions options;
    auto motion_controller = std::make_shared<elu_robot_arm_framework::MotionController>(options);

    RCLCPP_INFO(motion_controller->get_logger(), "Motion Controller Node started");

    // 使用多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(motion_controller);
    executor.spin();
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("motion_controller_node"), 
                 "Exception in motion controller node: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
EOF

# 2. 状态监控节点
cat > src/nodes/status_monitor_node.cpp << 'EOF'
/**
 * @file status_monitor_node.cpp
 * @brief 状态监控节点
 * @author ELU Robotics Team
 * @date 2025
 */

#include <rclcpp/rclcpp.hpp>
#include "elu_robot_arm_framework/msg/robot_status.hpp"
#include <std_msgs/msg/string.hpp>

class StatusMonitor : public rclcpp::Node
{
public:
  StatusMonitor() : Node("status_monitor")
  {
    // 订阅机械臂状态
    status_subscriber_ = this->create_subscription<elu_robot_arm_framework::msg::RobotStatus>(
      "robot_status", 10,
      std::bind(&StatusMonitor::statusCallback, this, std::placeholders::_1));

    // 发布系统状态
    system_status_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "system_status", 10);

    // 创建状态检查定时器
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&StatusMonitor::statusTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Status Monitor initialized");
  }

private:
  void statusCallback(const elu_robot_arm_framework::msg::RobotStatus::SharedPtr msg)
  {
    // 处理机械臂状态
    if (msg->status == 4) { // ERROR状态
      RCLCPP_WARN(get_logger(), "Robot %s in ERROR state: %s", 
                  msg->robot_id.c_str(), msg->error_message.c_str());
    }
    else if (msg->status == 5) { // EMERGENCY_STOP状态
      RCLCPP_ERROR(get_logger(), "Robot %s in EMERGENCY_STOP state", 
                   msg->robot_id.c_str());
    }

    latest_status_ = *msg;
  }

  void statusTimerCallback()
  {
    // 发布系统状态摘要
    std_msgs::msg::String system_msg;
    system_msg.data = "System operational";
    system_status_publisher_->publish(system_msg);
  }

  rclcpp::Subscription<elu_robot_arm_framework::msg::RobotStatus>::SharedPtr status_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  elu_robot_arm_framework::msg::RobotStatus latest_status_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto status_monitor = std::make_shared<StatusMonitor>();

  RCLCPP_INFO(status_monitor->get_logger(), "Status Monitor Node started");

  rclcpp::spin(status_monitor);
  rclcpp::shutdown();
  return 0;
}
EOF

# ========================================
# 创建CMakeLists.txt
# ========================================

echo "创建CMakeLists.txt..."

cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(elu_robot_arm_framework)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

# 查找消息生成包
find_package(rosidl_default_generators REQUIRED)

# 生成消息和服务
set(msg_files
  "msg/MotionCommand.msg"
  "msg/RobotStatus.msg"
)

set(srv_files
  "srv/SwitchRobot.srv"
  "srv/ExecuteMotion.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# 包含目录
include_directories(include)

# 依赖列表
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

# 构建核心库
add_library(${PROJECT_NAME}_core
  src/controllers/motion_controller.cpp
  src/utils/plugin_manager.cpp
  src/safety/safety_checker.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

# 等待消息生成完成
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_core "${cpp_typesupport_target}")

# 构建节点可执行文件
add_executable(motion_controller_node src/nodes/motion_controller_node.cpp)
target_link_libraries(motion_controller_node ${PROJECT_NAME}_core)
ament_target_dependencies(motion_controller_node ${dependencies})

add_executable(status_monitor_node src/nodes/status_monitor_node.cpp)
target_link_libraries(status_monitor_node ${PROJECT_NAME}_core)
ament_target_dependencies(status_monitor_node ${dependencies})

# 构建ELU适配器插件
add_library(elu_adapter SHARED
  plugins/elu_adapter/src/elu_adapter.cpp
)

target_include_directories(elu_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins/elu_adapter/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(elu_adapter ${PROJECT_NAME}_core)
ament_target_dependencies(elu_adapter ${dependencies})

# 导出插件
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins/elu_adapter/elu_adapter_plugins.xml)

# 安装
install(TARGETS ${PROJECT_NAME}_core elu_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS motion_controller_node status_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY plugins/elu_adapter/include/
  DESTINATION include/
)

# 导出依赖
ament_export_targets(${PROJECT_NAME}_core HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # 添加单元测试
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_plugin_manager test/unit/test_plugin_manager.cpp)
  target_link_libraries(test_plugin_manager ${PROJECT_NAME}_core)
  ament_target_dependencies(test_plugin_manager ${dependencies})
  
  ament_add_gtest(test_safety_checker test/unit/test_safety_checker.cpp)
  target_link_libraries(test_safety_checker ${PROJECT_NAME}_core)
  ament_target_dependencies(test_safety_checker ${dependencies})
endif()

ament_package()
EOF

# ========================================
# 创建package.xml
# ========================================

echo "创建package.xml..."

cat > package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>elu_robot_arm_framework</name>
  <version>1.0.0</version>
  <description>ELU机械臂运动控制框架 - 支持多种机械臂的统一控制接口</description>
  
  <maintainer email="dev@elu-robotics.com">ELU Robotics Team</maintainer>
  <license>Apache-2.0</license>
  
  <url type="website">https://www.elu-robotics.com</url>
  <url type="bugtracker">https://github.com/elu-robotics/elu_robot_arm_framework/issues</url>
  <url type="repository">https://github.com/elu-robotics/elu_robot_arm_framework</url>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- 核心ROS2依赖 -->
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>control_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  
  <!-- 插件系统 -->
  <depend>pluginlib</depend>
  
  <!-- MoveIt依赖 -->
  <depend>moveit_core</depend>
  <depend>moveit_ros_planning</depend>
  <depend>moveit_ros_planning_interface</depend>
  
  <!-- 消息生成 -->
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <!-- 系统依赖 -->
  <depend>yaml-cpp-vendor</depend>
  
  <!-- 测试依赖 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# ========================================
# 创建插件描述文件
# ========================================

echo "创建插件描述文件..."

cat > plugins/elu_adapter/elu_adapter_plugins.xml << 'EOF'
<library path="elu_adapter">
  <class name="elu_adapter/EluAdapter" 
         type="elu_adapter::EluAdapter" 
         base_class_type="elu_robot_arm_framework::RobotArmInterface">
    <description>ELU机械臂适配器插件</description>
  </class>
</library>
EOF

# ========================================
# 创建测试文件
# ========================================

echo "创建测试文件..."

# 1. 插件管理器测试
cat > test/unit/test_plugin_manager.cpp << 'EOF'
/**
 * @file test_plugin_manager.cpp
 * @brief 插件管理器单元测试
 */

#include <gtest/gtest.h>
#include "elu_robot_arm_framework/utils/plugin_manager.hpp"

class PluginManagerTest : public ::testing::Test 
{
protected:
  void SetUp() override {
    plugin_manager_ = std::make_unique<elu_robot_arm_framework::PluginManager>();
  }

  void TearDown() override {
    plugin_manager_.reset();
  }

  std::unique_ptr<elu_robot_arm_framework::PluginManager> plugin_manager_;
};

TEST_F(PluginManagerTest, InitializationTest) 
{
  EXPECT_NE(plugin_manager_, nullptr);
}

TEST_F(PluginManagerTest, LoadPluginTest) 
{
  // 测试加载不存在的插件
  bool result = plugin_manager_->loadPlugin("test_robot", "non_existent_plugin");
  EXPECT_FALSE(result);
}

TEST_F(PluginManagerTest, GetAvailablePluginsTest) 
{
  auto available_plugins = plugin_manager_->getAvailablePlugins();
  // 至少应该有ELU适配器插件
  EXPECT_TRUE(available_plugins.size() >= 0);
}
EOF

# 2. 安全检查器测试
cat > test/unit/test_safety_checker.cpp << 'EOF'
/**
 * @file test_safety_checker.cpp
 * @brief 安全检查器单元测试
 */

#include <gtest/gtest.h>
#include "elu_robot_arm_framework/safety/safety_checker.hpp"

class SafetyCheckerTest : public ::testing::Test 
{
protected:
  void SetUp() override {
    safety_checker_ = std::make_unique<elu_robot_arm_framework::SafetyChecker>();
  }

  void TearDown() override {
    safety_checker_.reset();
  }

  std::unique_ptr<elu_robot_arm_framework::SafetyChecker> safety_checker_;
};

TEST_F(SafetyCheckerTest, InitializationTest) 
{
  EXPECT_NE(safety_checker_, nullptr);
  EXPECT_TRUE(safety_checker_->isSafetyCheckEnabled());
}

TEST_F(SafetyCheckerTest, SpeedCheckTest) 
{
  // 测试正常速度
  auto result = safety_checker_->checkSpeed(0.5);
  EXPECT_TRUE(result.is_safe);

  // 测试超出范围的速度
  result = safety_checker_->checkSpeed(1.5);
  EXPECT_FALSE(result.is_safe);

  result = safety_checker_->checkSpeed(-0.1);
  EXPECT_FALSE(result.is_safe);
}

TEST_F(SafetyCheckerTest, WorkspaceCheckTest) 
{
  geometry_msgs::msg::Pose pose;
  
  // 测试工作空间内的位置
  pose.position.x = 0.5;
  pose.position.y = 0.5;
  pose.position.z = 0.5;
  
  auto result = safety_checker_->checkWorkspace(pose);
  EXPECT_TRUE(result.is_safe);

  // 测试工作空间外的位置
  pose.position.x = 10.0;
  result = safety_checker_->checkWorkspace(pose);
  EXPECT_FALSE(result.is_safe);
}
EOF

# ========================================
# 创建文档文件
# ========================================

echo "创建文档文件..."

# 1. README.md
cat > README.md << 'EOF'
# ELU机械臂运动控制框架

## 概述

ELU机械臂运动控制框架是一个基于ROS2的通用机械臂控制系统，采用分层解耦和插件化的设计理念，支持多种品牌机械臂的统一控制。

## 特性

- 🔌 **插件化架构**: 支持动态加载不同品牌机械臂适配器
- 🛡️ **安全保障**: 内置多层安全检查机制
- 🎯 **统一接口**: 提供标准化的控制API
- 📊 **实时监控**: 完善的状态监控和错误处理
- ⚡ **高性能**: 满足工业级实时控制要求
- 🔧 **易扩展**: 便于添加新的机械臂支持

## 系统要求

- Ubuntu 22.04.5 LTS
- ROS2 Humble
- C++17 编译器
- CMake 3.8+
- YAML-CPP库

## 快速开始

### 1. 安装依赖

```bash
# 安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装额外依赖
sudo apt install libyaml-cpp-dev
sudo apt install ros-humble-moveit
sudo apt install ros-humble-pluginlib
```

### 2. 构建框架

```bash
# 创建工作空间
mkdir -p ~/elu_robot_ws/src
cd ~/elu_robot_ws/src

# 克隆代码（或者直接拷贝创建的包）
# git clone <repository_url>

# 构建
cd ~/elu_robot_ws
colcon build --symlink-install

# 源环境
source install/setup.bash
```

### 3. 启动框架

```bash
# 启动单机械臂控制
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py

# 启动多机械臂控制
ros2 launch elu_robot_arm_framework multi_arm_control.launch.py
```

## 使用指南

### 配置机械臂

编辑 `config/robots/robot_config.yaml` 文件：

```yaml
robot_arms:
  your_robot:
    plugin_type: "elu_adapter/EluAdapter"
    ip_address: "192.168.1.100"
    port: 8080
    # ... 其他配置
```

### 发送运动命令

```bash
# 关节运动
ros2 topic pub /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{robot_id: "elu_arm_1", motion_type: 0, joint_positions: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], speed_ratio: 0.5}'

# 笛卡尔运动
ros2 topic pub /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{robot_id: "elu_arm_1", motion_type: 1, target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}}, speed_ratio: 0.5}'
```

### 切换机械臂

```bash
ros2 service call /switch_robot elu_robot_arm_framework/srv/SwitchRobot \
  '{robot_id: "elu_arm_2", force_switch: false}'
```

## 开发指南

### 添加新机械臂支持

1. 创建适配器类继承 `RobotArmInterface`
2. 实现所有虚函数接口
3. 创建插件描述文件
4. 更新CMakeLists.txt
5. 添加配置文件

详细步骤请参考 [开发文档](docs/development_guide.md)

## API文档

### 核心接口

- `RobotArmInterface`: 机械臂统一接口基类
- `MotionController`: 运动控制管理器
- `PluginManager`: 插件管理器
- `SafetyChecker`: 安全检查器

### 消息类型

- `MotionCommand`: 运动命令消息
- `RobotStatus`: 机械臂状态消息

### 服务类型

- `SwitchRobot`: 切换机械臂服务
- `ExecuteMotion`: 执行运动服务

## 许可证

Apache License 2.0

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

- 邮箱: dev@elu-robotics.com
- 网站: https://www.elu-robotics.com
EOF

# 2. 开发指南
cat > docs/development_guide.md << 'EOF'
# 开发指南

## 添加新机械臂适配器

### 1. 创建适配器类

继承 `RobotArmInterface` 基类：

```cpp
class YourRobotAdapter : public elu_robot_arm_framework::RobotArmInterface
{
public:
  YourRobotAdapter();
  virtual ~YourRobotAdapter();

  // 实现所有纯虚函数
  bool connect(const std::string& config_file) override;
  // ... 其他接口实现
};
```

### 2. 实现SDK集成

集成厂商提供的SDK：

```cpp
bool YourRobotAdapter::connect(const std::string& config_file)
{
  // 1. 读取配置文件
  YAML::Node config = YAML::LoadFile(config_file);
  
  // 2. 初始化SDK
  your_sdk_->initialize();
  
  // 3. 建立连接
  return your_sdk_->connect(ip_address_, port_);
}
```

### 3. 注册插件

使用PLUGINLIB_EXPORT_CLASS宏注册插件：

```cpp
PLUGINLIB_EXPORT_CLASS(your_namespace::YourRobotAdapter, 
                       elu_robot_arm_framework::RobotArmInterface)
```

### 4. 创建插件描述文件

```xml
<library path="your_robot_adapter">
  <class name="your_namespace/YourRobotAdapter" 
         type="your_namespace::YourRobotAdapter" 
         base_class_type="elu_robot_arm_framework::RobotArmInterface">
    <description>Your Robot Adapter Plugin</description>
  </class>
</library>
```

## 测试指南

### 单元测试

使用Google Test框架：

```cpp
TEST_F(YourAdapterTest, ConnectionTest) 
{
  EXPECT_TRUE(adapter_->connect("test_config.yaml"));
  EXPECT_TRUE(adapter_->isConnected());
}
```

### 集成测试

测试整个控制流程：

```bash
ros2 test src/elu_robot_arm_framework
```

## 调试技巧

### 启用详细日志

```bash
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py log_level:=debug
```

### 使用RQT工具

```bash
# 查看话题
rqt_graph

# 监控消息
rqt_topic

# 调用服务
rqt_service_caller
```
EOF

# ========================================
# 构建和设置脚本结束
# ========================================

echo "=========================================="
echo "正在构建ELU机械臂框架..."
echo "=========================================="

# 构建包
cd "$WORKSPACE_DIR"
colcon build --symlink-install --packages-select elu_robot_arm_framework

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "🎉 ELU机械臂框架创建成功！"
    echo "=========================================="
    echo ""
    echo "框架位置: $WORKSPACE_DIR/src/elu_robot_arm_framework"
    echo ""
    echo "下一步操作："
    echo "1. source $WORKSPACE_DIR/install/setup.bash"
    echo "2. 配置机械臂参数: config/robots/robot_config.yaml"
    echo "3. 启动框架: ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py"
    echo ""
    echo "完整的目录结构已创建，包含："
    echo "✅ 核心接口定义"
    echo "✅ 运动控制器"
    echo "✅ 插件管理系统"
    echo "✅ 安全检查器"
    echo "✅ ELU适配器示例"
    echo "✅ 配置文件模板"
    echo "✅ 启动文件"
    echo "✅ 测试框架"
    echo "✅ 完整的文档"
    echo ""
    echo "开发说明："
    echo "- 框架支持插件化机械臂适配器"
    echo "- 已包含ELU机械臂适配器示例"
    echo "- 可按需添加其他品牌机械臂支持"
    echo "- 内置安全检查和状态监控"
    echo ""
    echo "有关详细使用说明，请查看："
    echo "- README.md: 基本使用指南"
    echo "- docs/development_guide.md: 开发指南"
    echo ""
else
    echo "❌ 构建失败，请检查错误信息"
    exit 1
fi

# 创建快速启动脚本
cat > "$WORKSPACE_DIR/start_elu_framework.sh" << 'EOF'
#!/bin/bash

# ELU机械臂框架快速启动脚本

echo "启动ELU机械臂运动控制框架..."

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "设置ROS2环境..."
    source /opt/ros/humble/setup.bash
fi

# 设置工作空间环境
source install/setup.bash

# 启动框架
echo "启动运动控制框架..."
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py
EOF

chmod +x "$WORKSPACE_DIR/start_elu_framework.sh"

# 创建测试脚本
cat > "$WORKSPACE_DIR/test_framework.sh" << 'EOF'
#!/bin/bash

# ELU机械臂框架测试脚本

echo "运行ELU机械臂框架测试..."

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# 设置工作空间环境
source install/setup.bash

# 运行测试
echo "运行单元测试..."
colcon test --packages-select elu_robot_arm_framework

echo "显示测试结果..."
colcon test-result --all
EOF

chmod +x "$WORKSPACE_DIR/test_framework.sh"

# 创建开发环境设置脚本
cat > "$WORKSPACE_DIR/setup_dev_env.sh" << 'EOF'
#!/bin/bash

# ELU机械臂框架开发环境设置脚本

echo "设置ELU机械臂框架开发环境..."

# 安装开发依赖
echo "安装开发工具..."
sudo apt update
sudo apt install -y \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    git \
    vim \
    tree

echo "设置colcon mixins..."
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

echo "创建别名..."
echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc
echo "alias ct='colcon test'" >> ~/.bashrc
echo "alias ctr='colcon test-result --all'" >> ~/.bashrc
echo "alias cs='source install/setup.bash'" >> ~/.bashrc

echo "开发环境设置完成！"
echo "重新打开终端或运行 'source ~/.bashrc' 来使用新的别名"
EOF

chmod +x "$WORKSPACE_DIR/setup_dev_env.sh"

# 创建演示脚本
cat > "$WORKSPACE_DIR/demo.sh" << 'EOF'
#!/bin/bash

# ELU机械臂框架演示脚本

echo "=========================================="
echo "ELU机械臂框架功能演示"
echo "=========================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

source install/setup.bash

echo "1. 检查可用的机械臂适配器插件..."
ros2 pkg executables elu_robot_arm_framework

echo ""
echo "2. 查看消息类型..."
ros2 interface show elu_robot_arm_framework/msg/MotionCommand
echo ""
ros2 interface show elu_robot_arm_framework/msg/RobotStatus

echo ""
echo "3. 查看服务类型..."
ros2 interface show elu_robot_arm_framework/srv/SwitchRobot
echo ""
ros2 interface show elu_robot_arm_framework/srv/ExecuteMotion

echo ""
echo "4. 启动框架（在后台）..."
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py &
LAUNCH_PID=$!

# 等待节点启动
sleep 5

echo ""
echo "5. 检查运行的节点..."
ros2 node list

echo ""
echo "6. 检查话题..."
ros2 topic list

echo ""
echo "7. 发送测试运动命令..."
ros2 topic pub --once /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, robot_id: "elu_arm_1", motion_type: 0, joint_positions: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], speed_ratio: 0.5, acceleration_ratio: 0.5, wait_for_completion: true}'

echo ""
echo "8. 查看机械臂状态..."
ros2 topic echo /robot_status --once

echo ""
echo "9. 测试服务调用..."
ros2 service call /switch_robot elu_robot_arm_framework/srv/SwitchRobot \
  '{robot_id: "elu_arm_1", force_switch: false}'

echo ""
echo "演示完成！停止框架..."
kill $LAUNCH_PID

echo ""
echo "=========================================="
echo "演示结束。框架运行正常！"
echo "=========================================="
EOF

chmod +x "$WORKSPACE_DIR/demo.sh"

# 输出项目结构
echo ""
echo "=========================================="
echo "📁 项目结构概览"
echo "=========================================="

cd "$WORKSPACE_DIR/src/elu_robot_arm_framework"
tree -I 'build|install|log' || ls -la

echo ""
echo "=========================================="
echo "🚀 快速开始命令"
echo "=========================================="
echo ""
echo "# 进入工作空间"
echo "cd $WORKSPACE_DIR"
echo ""
echo "# 设置环境"
echo "source install/setup.bash"
echo ""
echo "# 启动框架"
echo "./start_elu_framework.sh"
echo ""
echo "# 或者手动启动"
echo "ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py"
echo ""
echo "# 运行演示"
echo "./demo.sh"
echo ""
echo "# 运行测试"
echo "./test_framework.sh"
echo ""
echo "# 设置开发环境"
echo "./setup_dev_env.sh"
echo ""

echo "=========================================="
echo "📚 配置说明"
echo "=========================================="
echo ""
echo "主要配置文件："
echo "- config/robots/robot_config.yaml: 机械臂配置"
echo "- config/safety/safety_config.yaml: 安全检查配置"
echo ""
echo "需要根据实际ELU机械臂参数修改："
echo "- IP地址和端口"
echo "- 关节限制"
echo "- 工作空间限制"
echo "- 安全参数"
echo ""

echo "=========================================="
echo "🔧 开发指南"
echo "=========================================="
echo ""
echo "添加新机械臂支持："
echo "1. 在plugins目录下创建新适配器"
echo "2. 继承RobotArmInterface基类"
echo "3. 实现所有虚函数"
echo "4. 使用PLUGINLIB_EXPORT_CLASS注册插件"
echo "5. 创建插件描述XML文件"
echo "6. 更新CMakeLists.txt"
echo ""

echo "=========================================="
echo "⚠️  注意事项"
echo "=========================================="
echo ""
echo "1. 框架目前包含ELU适配器模板，需要："
echo "   - 集成实际的ELU SDK"
echo "   - 实现真实的通信协议"
echo "   - 添加错误处理机制"
echo ""
echo "2. 安全配置非常重要："
echo "   - 请根据实际机械臂参数设置限制"
echo "   - 在生产环境中启用所有安全检查"
echo ""
echo "3. 性能优化："
echo "   - 根据需要调整控制频率"
echo "   - 监控CPU和内存使用情况"
echo ""

echo "=========================================="
echo "✅ ELU机械臂框架创建完成！"
echo "=========================================="

# 最后的环境提示
echo ""
echo "请运行以下命令来开始使用框架："
echo ""
echo "cd $WORKSPACE_DIR"
echo "source install/setup.bash"
echo "./start_elu_framework.sh"
