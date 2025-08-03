cmake_minimum_required(VERSION 3.8)
project(elu_robot_arm_framework)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++17标准（为了兼容机械臂SDK）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "The type of build, options are: Debug, Release, RelWithDebInfo, MinSizeRel." FORCE)
endif()

# 机械臂SDK相关选项
option(XCORE_LINK_SHARED_LIBS "Link shared library for xCore SDK" OFF)
option(XCORE_USE_XMATE_MODEL "Use xMateModel library for kinematics and dynamics" OFF)

# 查找ROS2依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs QUIET)

# 查找YAML-CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# 查找线程库（机械臂SDK需要）
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

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
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs trajectory_msgs
)

# 包含目录
include_directories(include)

# 添加机械臂SDK包含目录
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/xcore_sdk/include)
if(EXISTS ${SDK_INCLUDE_DIR})
    include_directories(${SDK_INCLUDE_DIR})
endif()

# ROS2依赖列表
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  pluginlib 
  trajectory_msgs
  std_srvs
)

if(control_msgs_FOUND)
  list(APPEND dependencies control_msgs)
endif()

# ==================== 机械臂SDK库设置 ====================

# 创建Eigen接口库（机械臂SDK依赖）
add_library(eigen INTERFACE)
target_compile_definitions(eigen INTERFACE EIGEN_DONT_ALIGN_STATICALLY)
target_include_directories(eigen INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen>)

# 机械臂SDK库路径设置
set(SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/xcore_sdk/lib)

# 创建机械臂SDK库目标
if(EXISTS ${SDK_LIB_DIR})
    # 导入的机械臂SDK库
    add_library(xcore_sdk_lib SHARED IMPORTED GLOBAL)
    add_library(xcore_sdk_static STATIC IMPORTED GLOBAL)
    
    if(WIN32)
        if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
            set(_ARCH_NAME "64bit")
        elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 4)
            set(_ARCH_NAME "32bit")
        endif()
        
        set_target_properties(xcore_sdk_lib PROPERTIES
            IMPORTED_IMPLIB_RELEASE ${SDK_LIB_DIR}/Windows/Release/${_ARCH_NAME}/xCoreSDK.lib
            IMPORTED_LOCATION_RELEASE ${SDK_LIB_DIR}/Windows/Release/${_ARCH_NAME}/xCoreSDK.dll
            IMPORTED_IMPLIB_DEBUG ${SDK_LIB_DIR}/Windows/Debug/${_ARCH_NAME}/xCoreSDK.lib
            IMPORTED_LOCATION_DEBUG ${SDK_LIB_DIR}/Windows/Debug/${_ARCH_NAME}/xCoreSDK.dll
        )
        set_target_properties(xcore_sdk_static PROPERTIES
            IMPORTED_LOCATION_RELEASE ${SDK_LIB_DIR}/Windows/Release/${_ARCH_NAME}/xCoreSDK_static.lib
            IMPORTED_LOCATION_DEBUG ${SDK_LIB_DIR}/Windows/Debug/${_ARCH_NAME}/xCoreSDK_static.lib
        )
    elseif(UNIX)
        set(XCORESDK_LIBRARY_DIR ${SDK_LIB_DIR}/Linux/${CMAKE_SYSTEM_PROCESSOR})
        set_target_properties(xcore_sdk_lib PROPERTIES
            IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.so
        )
        set_target_properties(xcore_sdk_static PROPERTIES
            IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.a
        )
    endif()
    
    # 创建别名用于链接
    if(XCORE_LINK_SHARED_LIBS)
        add_library(XCoreSDK::XCoreSDK ALIAS xcore_sdk_lib)
    else()
        add_library(XCoreSDK::XCoreSDK ALIAS xcore_sdk_static)
    endif()
endif()

# xMateModel库支持（可选）
if(XCORE_USE_XMATE_MODEL AND EXISTS ${SDK_LIB_DIR})
    add_library(xmatemodel_lib STATIC IMPORTED)
    
    if(WIN32)
        set_target_properties(xmatemodel_lib PROPERTIES
            IMPORTED_LOCATION_RELEASE ${SDK_LIB_DIR}/Windows/Release/64bit/xMateModel.lib
            IMPORTED_LOCATION_DEBUG ${SDK_LIB_DIR}/Windows/Debug/64bit/xMateModeld.lib)
    elseif(UNIX AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
        set_target_properties(xmatemodel_lib PROPERTIES
            IMPORTED_LOCATION ${SDK_LIB_DIR}/Linux/x86_64/libxMateModel.a)
    endif()
endif()

# ==================== 构建ROS2框架库 ====================

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

target_link_libraries(${PROJECT_NAME}_core 
  ${YAML_CPP_LIBRARIES}
  Threads::Threads
)

# 如果机械臂SDK可用，链接SDK库
if(TARGET XCoreSDK::XCoreSDK)
    target_link_libraries(${PROJECT_NAME}_core XCoreSDK::XCoreSDK eigen)
    target_compile_definitions(${PROJECT_NAME}_core PUBLIC 
        XCORE_SDK_AVAILABLE
        $<$<BOOL:${XCORE_LINK_SHARED_LIBS}>:XCORESDK_DLL>
        $<$<CXX_COMPILER_ID:MSVC>:_USE_MATH_DEFINES>
    )
    
    if(XCORE_USE_XMATE_MODEL AND TARGET xmatemodel_lib)
        target_link_libraries(${PROJECT_NAME}_core xmatemodel_lib)
        target_compile_definitions(${PROJECT_NAME}_core PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endif()
endif()

target_compile_options(${PROJECT_NAME}_core PUBLIC ${YAML_CPP_CFLAGS_OTHER})
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

# 等待消息生成完成
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_core "${cpp_typesupport_target}")


# 检查rokae_adapter.cpp是否存在
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/adapters/rokae_adapter.cpp)
    add_library(rokae_adapter SHARED
      src/adapters/rokae_adapter.cpp
    )
    
    target_include_directories(rokae_adapter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    
    target_link_libraries(rokae_adapter ${PROJECT_NAME}_core)
    
    if(TARGET XCoreSDK::XCoreSDK)
        target_link_libraries(rokae_adapter XCoreSDK::XCoreSDK eigen)
    endif()
    
    ament_target_dependencies(rokae_adapter ${dependencies})
    set(ROKAE_ADAPTER_AVAILABLE TRUE)
else()
    set(ROKAE_ADAPTER_AVAILABLE FALSE)
endif()

# ==================== 创建RokaeMotionPlanner库 ====================

# 检查是否存在RokaeMotionPlanner源文件
set(ROKAE_MOTION_PLANNER_SRC "")

# 首先检查用户指定的路径
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/planners/rokae_motion_planner.cpp)
    set(ROKAE_MOTION_PLANNER_SRC src/planners/rokae_motion_planner.cpp)
    message(STATUS "Found RokaeMotionPlanner at: src/planners/rokae_motion_planner.cpp")
else()
    message(WARNING "RokaeMotionPlanner source file not found. Checked paths:")
    message(WARNING "  - src/planners/rokae_motion_planner.cpp")
    message(WARNING "  - src/motion_planner/rokae_motion_planner.cpp")
    message(WARNING "Please ensure the file exists at one of these locations.")
endif()

# 只有在找到源文件时才创建库
if(NOT ROKAE_MOTION_PLANNER_SRC STREQUAL "")
    add_library(rokae_motion_planner SHARED
      ${ROKAE_MOTION_PLANNER_SRC}
    )
    
    target_include_directories(rokae_motion_planner PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    
    target_link_libraries(rokae_motion_planner ${PROJECT_NAME}_core)
    
    if(ROKAE_ADAPTER_AVAILABLE)
        target_link_libraries(rokae_motion_planner rokae_adapter)
    endif()
    
    if(TARGET XCoreSDK::XCoreSDK)
        target_link_libraries(rokae_motion_planner XCoreSDK::XCoreSDK eigen)
    endif()
    
    ament_target_dependencies(rokae_motion_planner ${dependencies})
    set(ROKAE_MOTION_PLANNER_AVAILABLE TRUE)
    message(STATUS "RokaeMotionPlanner library will be built")
else()
    set(ROKAE_MOTION_PLANNER_AVAILABLE FALSE)
    message(STATUS "RokaeMotionPlanner library will NOT be built - source file missing")
endif()

# ==================== 构建适配器插件 ====================

# ELU适配器插件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins/elu_adapter/src/elu_adapter.cpp)
    add_library(elu_adapter SHARED
      plugins/elu_adapter/src/elu_adapter.cpp
    )
    
    target_include_directories(elu_adapter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins/elu_adapter/include>
      $<INSTALL_INTERFACE:include>
    )
    
    target_link_libraries(elu_adapter ${PROJECT_NAME}_core)
    ament_target_dependencies(elu_adapter ${dependencies})
endif()

# ==================== 构建节点可执行文件 ====================

# motion_controller_node
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/motion_controller_node.cpp)
    add_executable(motion_controller_node src/nodes/motion_controller_node.cpp)
    target_link_libraries(motion_controller_node ${PROJECT_NAME}_core)
    ament_target_dependencies(motion_controller_node ${dependencies})
endif()

# status_monitor_node
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/status_monitor_node.cpp)
    add_executable(status_monitor_node src/nodes/status_monitor_node.cpp)
    target_link_libraries(status_monitor_node ${PROJECT_NAME}_core)
    ament_target_dependencies(status_monitor_node ${dependencies})
endif()

# rokae_integration_example - 修复链接问题
add_executable(rokae_integration_example src/nodes/rokae_integration_example.cpp)

# 链接所有必要的库（按正确顺序）
target_link_libraries(rokae_integration_example 
  ${PROJECT_NAME}_core
)

# 条件链接可选库
if(ROKAE_MOTION_PLANNER_AVAILABLE)
    target_link_libraries(rokae_integration_example rokae_motion_planner)
endif()

if(ROKAE_ADAPTER_AVAILABLE)
    target_link_libraries(rokae_integration_example rokae_adapter)
endif()

# 如果SDK可用，直接链接SDK
if(TARGET XCoreSDK::XCoreSDK)
    target_link_libraries(rokae_integration_example XCoreSDK::XCoreSDK eigen)
endif()

ament_target_dependencies(rokae_integration_example ${dependencies})

# ==================== 安装配置 ====================

# 收集所有目标
set(INSTALL_TARGETS ${PROJECT_NAME}_core)

if(ROKAE_ADAPTER_AVAILABLE)
    list(APPEND INSTALL_TARGETS rokae_adapter)
endif()

if(ROKAE_MOTION_PLANNER_AVAILABLE)
    list(APPEND INSTALL_TARGETS rokae_motion_planner)
endif()

if(TARGET elu_adapter)
    list(APPEND INSTALL_TARGETS elu_adapter)
endif()

# 安装库和插件
install(TARGETS ${INSTALL_TARGETS}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装可执行文件
set(EXECUTABLE_TARGETS rokae_integration_example)

if(TARGET motion_controller_node)
    list(APPEND EXECUTABLE_TARGETS motion_controller_node)
endif()

if(TARGET status_monitor_node)
    list(APPEND EXECUTABLE_TARGETS status_monitor_node)
endif()

install(TARGETS ${EXECUTABLE_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins/elu_adapter/include/)
    install(DIRECTORY plugins/elu_adapter/include/
      DESTINATION include/
    )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins/rokae_adapter/include/)
    install(DIRECTORY plugins/rokae_adapter/include/
      DESTINATION include/
    )
endif()

# 安装配置和启动文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
    install(DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}/
    )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
    install(DIRECTORY config
      DESTINATION share/${PROJECT_NAME}/
    )
endif()

# 导出插件（如果插件文件存在）
if(TARGET elu_adapter AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins/elu_adapter/elu_adapter_plugins.xml)
    pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins/elu_adapter/elu_adapter_plugins.xml)
endif()

if(ROKAE_ADAPTER_AVAILABLE AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins/rokae_adapter/rokae_adapter_plugins.xml)
    pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins/rokae_adapter/rokae_adapter_plugins.xml)
endif()

# 导出目标和依赖
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# ==================== 测试配置 ====================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # 只有在有gtest的情况下才添加测试
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_plugin_manager.cpp)
        ament_add_gtest(test_plugin_manager test/unit/test_plugin_manager.cpp)
        target_link_libraries(test_plugin_manager ${PROJECT_NAME}_core)
        ament_target_dependencies(test_plugin_manager ${dependencies})
    endif()
    
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_safety_checker.cpp)
        ament_add_gtest(test_safety_checker test/unit/test_safety_checker.cpp)
        target_link_libraries(test_safety_checker ${PROJECT_NAME}_core)
        ament_target_dependencies(test_safety_checker ${dependencies})
    endif()
    
    # 如果机械臂SDK可用，添加SDK相关测试
    if(TARGET XCoreSDK::XCoreSDK AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_rokae_adapter.cpp)
        ament_add_gtest(test_rokae_adapter test/unit/test_rokae_adapter.cpp)
        target_link_libraries(test_rokae_adapter ${PROJECT_NAME}_core XCoreSDK::XCoreSDK eigen)
        ament_target_dependencies(test_rokae_adapter ${dependencies})
    endif()
  endif()
endif()

ament_package()