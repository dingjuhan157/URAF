#!/bin/bash

# 修复ELU机械臂框架依赖问题脚本
# 适用于 Ubuntu 22.04.5 + ROS2 Humble

set -e

echo "=========================================="
echo "修复ELU机械臂框架依赖问题"
echo "=========================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置，请先source ROS2环境"
    echo "请执行: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "检测到ROS2版本: $ROS_DISTRO"

# 更新包管理器
echo "更新包管理器..."
sudo apt update

# 安装缺失的ROS2包
echo "安装缺失的ROS2包依赖..."
sudo apt install -y \
    ros-humble-control-msgs \
    ros-humble-controller-manager \
    ros-humble-controller-interface \
    ros-humble-hardware-interface \
    ros-humble-realtime-tools \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit-msgs \
    ros-humble-moveit-resources \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-configs-utils \
    ros-humble-warehouse-ros \
    ros-humble-pilz-industrial-motion-planner \
    ros-humble-moveit-servo \
    ros-humble-geometric-shapes \
    ros-humble-random-numbers \
    ros-humble-srdfdom \
    ros-humble-ament-cmake-gtest \
    ros-humble-rqt-robot-monitor \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-urdf \
    ros-humble-xacro

# 安装YAML-CPP开发包
echo "安装YAML-CPP开发包..."
sudo apt install -y \
    libyaml-cpp-dev \
    libyaml-cpp0.7

# 检查pluginlib是否已安装
echo "检查pluginlib包..."
ros2 pkg list | grep pluginlib || sudo apt install -y ros-humble-pluginlib

# 设置工作空间
WORKSPACE_DIR="$HOME/elu_robot_ws"
cd "$WORKSPACE_DIR"

# 更新CMakeLists.txt以修复依赖问题
echo "更新CMakeLists.txt..."
cat > src/elu_robot_arm_framework/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(elu_robot_arm_framework)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包 - 按可用性分组
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

# 可选的高级包 - 如果找不到会跳过
find_package(trajectory_msgs QUIET)
find_package(control_msgs QUIET)
find_package(moveit_core QUIET)
find_package(moveit_ros_planning QUIET)
find_package(moveit_ros_planning_interface QUIET)

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

# 基础依赖列表
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
  add_definitions(-DHAS_TRAJECTORY_MSGS)
endif()

if(control_msgs_FOUND)
  list(APPEND dependencies control_msgs)
  add_definitions(-DHAS_CONTROL_MSGS)
endif()

if(moveit_core_FOUND AND moveit_ros_planning_FOUND)
  list(APPEND dependencies moveit_core moveit_ros_planning)
  add_definitions(-DHAS_MOVEIT)
endif()

# 构建核心库
add_library(${PROJECT_NAME}_core
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

install(DIRECTORY plugins/elu_adapter/include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# 导出依赖
ament_export_targets(${PROJECT_NAME}_core HAS_LIBRARY_TARGET)
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

# 更新package.xml以修复依赖
echo "更新package.xml..."
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
  
  <!-- 可选的高级依赖 -->
  <depend condition="$ROS_DISTRO == humble">trajectory_msgs</depend>
  <depend condition="$ROS_DISTRO == humble">control_msgs</depend>
  
  <!-- MoveIt依赖（可选） -->
  <depend condition="$ROS_DISTRO == humble">moveit_core</depend>
  <depend condition="$ROS_DISTRO == humble">moveit_ros_planning</depend>
  <depend condition="$ROS_DISTRO == humble">moveit_ros_planning_interface</depend>
  
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

# 更新头文件以处理可选依赖
echo "更新头文件以处理可选依赖..."

# 更新运动控制器头文件
cat > src/elu_robot_arm_framework/include/elu_robot_arm_framework/controllers/motion_controller.hpp << 'EOF'
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
#include "elu_robot_arm_framework/srv/switch_robot.hpp"
#include "elu_robot_arm_framework/srv/execute_motion.hpp"

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

# 更新启动文件，简化依赖
echo "更新启动文件..."
cat > src/elu_robot_arm_framework/launch/elu_robot_framework.launch.py << 'EOF'
#!/usr/bin/env python3
"""
ELU机械臂运动控制框架启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            {
                'robot_config': LaunchConfiguration('robot_config'),
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

# 清理之前的构建缓存
echo "清理构建缓存..."
rm -rf build/ install/ log/

# 重新构建
echo "重新构建ELU机械臂框架..."
colcon build --symlink-install --packages-select elu_robot_arm_framework

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "🎉 ELU机械臂框架修复并构建成功！"
    echo "=========================================="
    echo ""
    echo "修复的问题："
    echo "✅ 安装了缺失的ROS2包依赖"
    echo "✅ 更新了CMakeLists.txt以处理可选依赖"
    echo "✅ 修复了package.xml依赖声明"
    echo "✅ 简化了启动文件配置"
    echo ""
    echo "现在可以正常使用框架了："
    echo "source install/setup.bash"
    echo "./start_elu_framework.sh"
    echo ""
else
    echo "❌ 构建仍然失败，请检查错误信息"
    echo ""
    echo "可能的解决方案："
    echo "1. 检查是否所有ROS2包都已正确安装"
    echo "2. 确保YAML-CPP库已安装: sudo apt install libyaml-cpp-dev"
    echo "3. 重新source ROS2环境: source /opt/ros/humble/setup.bash"
    echo "4. 尝试单独安装MoveIt: sudo apt install ros-humble-moveit"
    exit 1
fi

echo ""
echo "=========================================="
echo "📦 已安装的依赖包"
echo "=========================================="
echo "- control_msgs"
echo "- moveit相关包"
echo "- yaml-cpp开发库"
echo "- pluginlib"
echo "- 其他必要的ROS2包"
echo ""

echo "=========================================="
echo "🔧 如果仍有问题，请尝试："
echo "=========================================="
echo "1. 更新所有ROS2包:"
echo "   sudo apt update && sudo apt upgrade"
echo ""
echo "2. 重新安装MoveIt:"
echo "   sudo apt install --reinstall ros-humble-moveit"
echo ""
echo "3. 检查环境变量:"
echo "   echo \$ROS_DISTRO"
echo "   echo \$ROS_VERSION"
echo ""
echo "4. 清理并重新构建:"
echo "   rm -rf build/ install/ log/"
echo "   colcon build --symlink-install"
echo ""
