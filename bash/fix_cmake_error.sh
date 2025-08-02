#!/bin/bash

# 修复ELU机械臂框架CMake导出错误脚本
# 适用于 Ubuntu 22.04.5 + ROS2 Humble

set -e

echo "=========================================="
echo "修复ELU机械臂框架CMake配置错误"
echo "=========================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置，请先source ROS2环境"
    echo "请执行: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "检测到ROS2版本: $ROS_DISTRO"

# 设置工作空间
WORKSPACE_DIR="$HOME/elu_robot_ws"
cd "$WORKSPACE_DIR"

# 先安装可能缺失的依赖
echo "安装可能缺失的依赖..."
sudo apt update
sudo apt install -y \
    libyaml-cpp-dev \
    ros-humble-pluginlib \
    ros-humble-ament-cmake-gtest \
    ros-humble-controller-manager \
    ros-humble-trajectory-msgs \
    ros-humble-control-msgs

echo "修复CMakeLists.txt中的导出问题..."

# 创建修复后的CMakeLists.txt
cat > src/elu_robot_arm_framework/CMakeLists.txt << 'EOF'
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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

# 可选依赖
find_package(trajectory_msgs QUIET)
find_package(control_msgs QUIET)

# 查找YAML-CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

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
  tf2
  tf2_ros
  pluginlib
)

# 添加可选依赖
if(trajectory_msgs_FOUND)
  list(APPEND dependencies trajectory_msgs)
endif()

if(control_msgs_FOUND)
  list(APPEND dependencies control_msgs)
endif()

# 构建核心库
add_library(${PROJECT_NAME}_core SHARED
  src/controllers/motion_controller.cpp
  src/utils/plugin_manager.cpp
  src/safety/safety_checker.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_core ${YAML_CPP_LIBRARIES})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${YAML_CPP_CFLAGS_OTHER})

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

# 安装目标
install(TARGETS ${PROJECT_NAME}_core elu_adapter
  EXPORT export_${PROJECT_NAME}
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

install(DIRECTORY plugins/elu_adapter/include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# 导出目标和依赖
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # 只有在有gtest的情况下才添加测试
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_plugin_manager test/unit/test_plugin_manager.cpp)
    target_link_libraries(test_plugin_manager ${PROJECT_NAME}_core)
    ament_target_dependencies(test_plugin_manager ${dependencies})
    
    ament_add_gtest(test_safety_checker test/unit/test_safety_checker.cpp)
    target_link_libraries(test_safety_checker ${PROJECT_NAME}_core)
    ament_target_dependencies(test_safety_checker ${dependencies})
  endif()
endif()

ament_package()
EOF

echo "修复package.xml..."

cat > src/elu_robot_arm_framework/package.xml << 'EOF'
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
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>pluginlib</depend>
  
  <!-- 可选依赖 -->
  <depend>trajectory_msgs</depend>
  <depend>control_msgs</depend>
  
  <!-- 消息生成 -->
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <!-- 测试依赖 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo "简化服务定义以避免循环依赖..."

# 更新服务定义
cat > src/elu_robot_arm_framework/srv/ExecuteMotion.srv << 'EOF'
# 执行运动服务定义

# 请求
string robot_id
int32 motion_type
float64[] joint_positions
geometry_msgs/Pose target_pose
float64 speed_ratio
float64 acceleration_ratio
bool wait_for_completion

---

# 响应
bool success
string message
float64 execution_time
EOF

echo "更新运动控制器源码以修复编译问题..."

cat > src/elu_robot_arm_framework/src/controllers/motion_controller.cpp << 'EOF'
/**
 * @file motion_controller.cpp
 * @brief 运动控制管理器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_robot_arm_framework/controllers/motion_controller.hpp"
#include <fstream>

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
    // 简化配置读取 - 避免yaml-cpp依赖问题
    RCLCPP_INFO(get_logger(), "Loading robot: %s from config: %s", 
                robot_id.c_str(), config_file.c_str());

    // 默认使用ELU适配器
    std::string plugin_name = "elu_adapter/EluAdapter";

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

bool MotionController::unloadRobot(const std::string& robot_id)
{
  auto it = loaded_robots_.find(robot_id);
  if (it == loaded_robots_.end()) {
    return false;
  }

  if (it->second && it->second->isConnected()) {
    it->second->disconnect();
  }

  loaded_robots_.erase(it);
  
  if (current_robot_id_ == robot_id) {
    current_robot_.reset();
    current_robot_id_.clear();
  }

  return plugin_manager_->unloadPlugin(robot_id);
}

std::string MotionController::getCurrentRobotId() const
{
  return current_robot_id_;
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

bool MotionController::emergencyStop()
{
  if (current_robot_) {
    return current_robot_->emergencyStop();
  }
  return false;
}

bool MotionController::clearError()
{
  if (current_robot_) {
    return current_robot_->clearError();
  }
  return false;
}

msg::RobotStatus MotionController::getRobotStatus(const std::string& robot_id)
{
  msg::RobotStatus status;
  
  std::string target_id = robot_id.empty() ? current_robot_id_ : robot_id;
  
  if (target_id.empty() || loaded_robots_.find(target_id) == loaded_robots_.end()) {
    status.robot_id = target_id;
    status.is_connected = false;
    status.status = static_cast<int32_t>(RobotState::DISCONNECTED);
    return status;
  }

  auto robot = loaded_robots_[target_id];
  status.robot_id = target_id;
  status.robot_model = robot->getRobotModel();
  status.is_connected = robot->isConnected();
  status.status = static_cast<int32_t>(robot->getStatus());
  status.current_joints = robot->getCurrentJoints();
  status.current_pose = robot->getCurrentPose();
  status.error_message = robot->getErrorMessage();

  return status;
}

std::vector<std::string> MotionController::getLoadedRobots() const
{
  std::vector<std::string> robots;
  for (const auto& [robot_id, robot] : loaded_robots_) {
    robots.push_back(robot_id);
  }
  return robots;
}

void MotionController::motionCommandCallback(const msg::MotionCommand::SharedPtr msg)
{
  if (!validateMotionCommand(*msg)) {
    RCLCPP_WARN(get_logger(), "Invalid motion command received");
    return;
  }

  executeMotion(*msg);
}

void MotionController::switchRobotCallback(
  const std::shared_ptr<srv::SwitchRobot::Request> request,
  std::shared_ptr<srv::SwitchRobot::Response> response)
{
  response->previous_robot = current_robot_id_;
  response->success = switchRobot(request->robot_id);
  
  if (response->success) {
    response->message = "Successfully switched to robot: " + request->robot_id;
  } else {
    response->message = "Failed to switch to robot: " + request->robot_id;
  }
}

void MotionController::executeMotionCallback(
  const std::shared_ptr<srv::ExecuteMotion::Request> request,
  std::shared_ptr<srv::ExecuteMotion::Response> response)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  // 创建运动命令
  msg::MotionCommand command;
  command.robot_id = request->robot_id;
  command.motion_type = request->motion_type;
  command.joint_positions = request->joint_positions;
  command.target_pose = request->target_pose;
  command.speed_ratio = request->speed_ratio;
  command.acceleration_ratio = request->acceleration_ratio;
  command.wait_for_completion = request->wait_for_completion;

  response->success = executeMotion(command);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  response->execution_time = duration.count() / 1000.0;

  if (response->success) {
    response->message = "Motion executed successfully";
  } else {
    response->message = "Motion execution failed";
  }
}

void MotionController::statusTimerCallback()
{
  if (!current_robot_) {
    return;
  }

  // 发布机械臂状态
  auto status_msg = getRobotStatus();
  status_msg.header.stamp = this->now();
  status_publisher_->publish(status_msg);
}

void MotionController::loadRobotConfigs()
{
  // 简化配置加载 - 使用默认配置
  RCLCPP_INFO(get_logger(), "Loading default robot configurations...");
  
  // 自动加载默认ELU机械臂
  std::string default_config = "config/robots/robot_config.yaml";
  loadRobot("elu_arm_1", default_config);
  switchRobot("elu_arm_1");
}

bool MotionController::validateMotionCommand(const msg::MotionCommand& command)
{
  // 基本验证
  if (command.speed_ratio < 0.0 || command.speed_ratio > 1.0) {
    RCLCPP_WARN(get_logger(), "Invalid speed ratio: %f", command.speed_ratio);
    return false;
  }

  if (command.acceleration_ratio < 0.0 || command.acceleration_ratio > 1.0) {
    RCLCPP_WARN(get_logger(), "Invalid acceleration ratio: %f", command.acceleration_ratio);
    return false;
  }

  return true;
}

} // namespace elu_robot_arm_framework
EOF

echo "清理之前的构建缓存..."
rm -rf build/ install/ log/

echo "重新构建ELU机械臂框架..."
colcon build --symlink-install --packages-select elu_robot_arm_framework --cmake-clean-cache

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "🎉 ELU机械臂框架修复并构建成功！"
    echo "=========================================="
    echo ""
    echo "修复的问题："
    echo "✅ 修复了CMake导出目标错误"
    echo "✅ 简化了服务定义避免循环依赖"
    echo "✅ 更新了package.xml中的包名错误"
    echo "✅ 简化了YAML配置依赖"
    echo "✅ 修复了库的导出声明"
    echo ""
    echo "现在可以正常使用框架了："
    echo "source install/setup.bash"
    echo "./start_elu_framework.sh"
    echo ""
    
    # 创建简化的测试脚本
    cat > test_basic_functions.sh << 'EOF'
#!/bin/bash

echo "测试ELU机械臂框架基本功能..."

# 设置环境
source install/setup.bash

# 检查包是否正确安装
echo "1. 检查包安装状态..."
ros2 pkg list | grep elu_robot_arm_framework

echo "2. 检查可执行文件..."
ros2 pkg executables elu_robot_arm_framework

echo "3. 检查消息类型..."
ros2 interface list | grep elu_robot_arm_framework

echo "4. 启动框架（后台运行3秒）..."
timeout 3s ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py || true

echo "基本功能测试完成！"
EOF
    
    chmod +x test_basic_functions.sh
    
    echo "运行基本功能测试..."
    ./test_basic_functions.sh
    
else
    echo "❌ 构建仍然失败，请检查详细错误信息"
    echo ""
    echo "可能的解决方案："
    echo "1. 检查ROS2环境: echo \$ROS_DISTRO"
    echo "2. 重新安装ament-cmake: sudo apt install ros-humble-ament-cmake"
    echo "3. 清理并重试: rm -rf build/ install/ log/ && colcon build"
    exit 1
fi

echo ""
echo "=========================================="
echo "🎯 下一步操作指南"
echo "=========================================="
echo ""
echo "1. 设置环境:"
echo "   source install/setup.bash"
echo ""
echo "2. 启动框架:"
echo "   ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py"
echo ""
echo "3. 发送测试命令:"
echo "   ros2 topic pub /motion_command elu_robot_arm_framework/msg/MotionCommand '{robot_id: \"elu_arm_1\", motion_type: 0, joint_positions: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], speed_ratio: 0.5}'"
echo ""
echo "4. 查看状态:"
echo "   ros2 topic echo /robot_status"
echo ""
