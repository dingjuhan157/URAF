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
