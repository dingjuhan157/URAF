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
option(BUILD_VISION_SUPPORT "Build vision communication support" ON)

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
  "msg/VisionResult.msg"
)

set(srv_files
  "srv/SwitchRobot.srv"
  "srv/ExecuteMotion.srv"
  "srv/TriggerVision.srv"
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
    set(XCORE_SDK_FOUND TRUE)
    message(STATUS "Found XCore SDK headers at: ${SDK_INCLUDE_DIR}")
else()
    set(XCORE_SDK_FOUND FALSE)
    message(STATUS "XCore SDK headers not found - some features will be disabled")
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
if(EXISTS ${SDK_LIB_DIR} AND XCORE_SDK_FOUND)
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
        if(EXISTS ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.so)
            set_target_properties(xcore_sdk_lib PROPERTIES
                IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.so
            )
        endif()
        if(EXISTS ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.a)
            set_target_properties(xcore_sdk_static PROPERTIES
                IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK.a
            )
        endif()
    endif()
    
    # 创建别名用于链接
    if(XCORE_LINK_SHARED_LIBS)
        add_library(XCoreSDK::XCoreSDK ALIAS xcore_sdk_lib)
    else()
        add_library(XCoreSDK::XCoreSDK ALIAS xcore_sdk_static)
    endif()
    
    set(XCORE_SDK_LIBRARY_FOUND TRUE)
    message(STATUS "XCore SDK libraries configured")
else()
    set(XCORE_SDK_LIBRARY_FOUND FALSE)
    message(STATUS "XCore SDK libraries not found - building without SDK support")
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
    
    set(XMATEMODEL_AVAILABLE TRUE)
else()
    set(XMATEMODEL_AVAILABLE FALSE)
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
if(XCORE_SDK_LIBRARY_FOUND)
    target_link_libraries(${PROJECT_NAME}_core XCoreSDK::XCoreSDK eigen)
    target_compile_definitions(${PROJECT_NAME}_core PUBLIC 
        XCORE_SDK_AVAILABLE
        $<$<BOOL:${XCORE_LINK_SHARED_LIBS}>:XCORESDK_DLL>
        $<$<CXX_COMPILER_ID:MSVC>:_USE_MATH_DEFINES>
    )
    
    if(XMATEMODEL_AVAILABLE)
        target_link_libraries(${PROJECT_NAME}_core xmatemodel_lib)
        target_compile_definitions(${PROJECT_NAME}_core PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endif()
endif()

target_compile_options(${PROJECT_NAME}_core PUBLIC ${YAML_CPP_CFLAGS_OTHER})
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

# 等待消息生成完成
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_core "${cpp_typesupport_target}")

# ==================== 构建Rokae适配器 ====================

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
    
    if(XCORE_SDK_LIBRARY_FOUND)
        target_link_libraries(rokae_adapter XCoreSDK::XCoreSDK eigen)
    endif()
    
    ament_target_dependencies(rokae_adapter ${dependencies})
    set(ROKAE_ADAPTER_AVAILABLE TRUE)
    message(STATUS "RokaeAdapter library will be built")
else()
    set(ROKAE_ADAPTER_AVAILABLE FALSE)
    message(STATUS "RokaeAdapter library will NOT be built - source file missing")
    message(STATUS "Expected location: src/adapters/rokae_adapter.cpp")
endif()

# ==================== 构建RokaeMotionPlanner库 ====================

# 检查是否存在RokaeMotionPlanner源文件
set(ROKAE_MOTION_PLANNER_SRC "")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/planners/rokae_motion_planner.cpp)
    set(ROKAE_MOTION_PLANNER_SRC src/planners/rokae_motion_planner.cpp)
    message(STATUS "Found RokaeMotionPlanner at: src/planners/rokae_motion_planner.cpp")
elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_planner/rokae_motion_planner.cpp)
    set(ROKAE_MOTION_PLANNER_SRC src/motion_planner/rokae_motion_planner.cpp)
    message(STATUS "Found RokaeMotionPlanner at: src/motion_planner/rokae_motion_planner.cpp")
else()
    message(STATUS "RokaeMotionPlanner source file not found")
    message(STATUS "Checked paths:")
    message(STATUS "  - src/planners/rokae_motion_planner.cpp")
    message(STATUS "  - src/motion_planner/rokae_motion_planner.cpp")
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
    
    if(XCORE_SDK_LIBRARY_FOUND)
        target_link_libraries(rokae_motion_planner XCoreSDK::XCoreSDK eigen)
    endif()
    
    ament_target_dependencies(rokae_motion_planner ${dependencies})
    set(ROKAE_MOTION_PLANNER_AVAILABLE TRUE)
    message(STATUS "RokaeMotionPlanner library will be built")
else()
    set(ROKAE_MOTION_PLANNER_AVAILABLE FALSE)
    message(STATUS "RokaeMotionPlanner library will NOT be built - source file missing")
endif()

# ==================== 构建视觉通信模块 ====================

if(BUILD_VISION_SUPPORT)
    # 检查视觉通信源文件是否存在
    set(VISION_COMM_SOURCES "")
    
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/vision/vision_communication.cpp)
        list(APPEND VISION_COMM_SOURCES src/vision/vision_communication.cpp)
        message(STATUS "Found VisionCommunication at: src/vision/vision_communication.cpp")
    elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/communication/vision_communication.cpp)
        list(APPEND VISION_COMM_SOURCES src/communication/vision_communication.cpp)
        message(STATUS "Found VisionCommunication at: src/communication/vision_communication.cpp")
    else()
        message(STATUS "VisionCommunication source file not found")
        message(STATUS "Checked paths:")
        message(STATUS "  - src/vision/vision_communication.cpp")
        message(STATUS "  - src/communication/vision_communication.cpp")
    endif()
    
    if(NOT VISION_COMM_SOURCES STREQUAL "")
        # 构建视觉通信库
        add_library(vision_communication SHARED
          ${VISION_COMM_SOURCES}
        )
        
        target_include_directories(vision_communication PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
        )
        
        target_link_libraries(vision_communication ${PROJECT_NAME}_core)
        
        # 如果Rokae适配器可用，链接它
        if(ROKAE_ADAPTER_AVAILABLE)
            target_link_libraries(vision_communication rokae_adapter)
        endif()
        
        # 网络库支持
        if(WIN32)
            target_link_libraries(vision_communication ws2_32)
            target_compile_definitions(vision_communication PRIVATE
                WIN32_LEAN_AND_MEAN
                _WIN32_WINNT=0x0601  # Windows 7+
            )
        else()
            # Linux下可能需要的网络库
            find_package(PkgConfig QUIET)
            if(PkgConfig_FOUND)
                pkg_check_modules(LIBUDEV libudev)
                if(LIBUDEV_FOUND)
                    target_link_libraries(vision_communication ${LIBUDEV_LIBRARIES})
                    target_include_directories(vision_communication PRIVATE ${LIBUDEV_INCLUDE_DIRS})
                endif()
            endif()
        endif()
        
        ament_target_dependencies(vision_communication ${dependencies})
        set(VISION_COMMUNICATION_AVAILABLE TRUE)
        message(STATUS "VisionCommunication library will be built")
    else()
        set(VISION_COMMUNICATION_AVAILABLE FALSE)
        message(STATUS "VisionCommunication library will NOT be built - source file missing")
    endif()
else()
    set(VISION_COMMUNICATION_AVAILABLE FALSE)
    message(STATUS "Vision communication support disabled by BUILD_VISION_SUPPORT=OFF")
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
    set(ELU_ADAPTER_AVAILABLE TRUE)
else()
    set(ELU_ADAPTER_AVAILABLE FALSE)
endif()

# ==================== 构建节点可执行文件 ====================

# motion_controller_node  
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/motion_controller_node.cpp)
    add_executable(motion_controller_node src/nodes/motion_controller_node.cpp)
    target_link_libraries(motion_controller_node ${PROJECT_NAME}_core)
    ament_target_dependencies(motion_controller_node ${dependencies})
    set(MOTION_CONTROLLER_NODE_AVAILABLE TRUE)
else()
    set(MOTION_CONTROLLER_NODE_AVAILABLE FALSE)
endif()

# status_monitor_node
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/status_monitor_node.cpp)
    add_executable(status_monitor_node src/nodes/status_monitor_node.cpp)
    target_link_libraries(status_monitor_node ${PROJECT_NAME}_core)
    ament_target_dependencies(status_monitor_node ${dependencies})
    set(STATUS_MONITOR_NODE_AVAILABLE TRUE)
else()
    set(STATUS_MONITOR_NODE_AVAILABLE FALSE)
endif()

# rokae_integration_example
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/rokae_integration_example.cpp)
    add_executable(rokae_integration_example src/nodes/rokae_integration_example.cpp)
    
    # 链接所有必要的库（按正确顺序）
    target_link_libraries(rokae_integration_example ${PROJECT_NAME}_core)
    
    # 条件链接可选库
    if(ROKAE_MOTION_PLANNER_AVAILABLE)
        target_link_libraries(rokae_integration_example rokae_motion_planner)
    endif()
    
    if(ROKAE_ADAPTER_AVAILABLE)
        target_link_libraries(rokae_integration_example rokae_adapter)
    endif()
    
    # 如果SDK可用，直接链接SDK
    if(XCORE_SDK_LIBRARY_FOUND)
        target_link_libraries(rokae_integration_example XCoreSDK::XCoreSDK eigen)
    endif()
    
    ament_target_dependencies(rokae_integration_example ${dependencies})
    set(ROKAE_INTEGRATION_EXAMPLE_AVAILABLE TRUE)
else()
    set(ROKAE_INTEGRATION_EXAMPLE_AVAILABLE FALSE)
endif()

# ==================== 视觉通信节点 ====================

# 独立的视觉通信节点
if(VISION_COMMUNICATION_AVAILABLE AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/vision_communication_node.cpp)
    add_executable(vision_communication_node 
      src/nodes/vision_communication_node.cpp
    )
    
    target_link_libraries(vision_communication_node 
      ${PROJECT_NAME}_core
      vision_communication
    )
    
    if(ROKAE_ADAPTER_AVAILABLE)
        target_link_libraries(vision_communication_node rokae_adapter)
    endif()
    
    # Windows网络库支持
    if(WIN32)
        target_link_libraries(vision_communication_node ws2_32)
    endif()
    
    ament_target_dependencies(vision_communication_node ${dependencies})
    set(VISION_COMMUNICATION_NODE_AVAILABLE TRUE)
else()
    set(VISION_COMMUNICATION_NODE_AVAILABLE FALSE)
endif()

# 视觉集成示例节点
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/vision_integration_example.cpp)
    add_executable(vision_integration_example 
      src/nodes/vision_integration_example.cpp
    )
    
    target_link_libraries(vision_integration_example ${PROJECT_NAME}_core)
    
    # 条件链接可选库
    if(VISION_COMMUNICATION_AVAILABLE)
        target_link_libraries(vision_integration_example vision_communication)
    endif()
    
    if(ROKAE_ADAPTER_AVAILABLE)
        target_link_libraries(vision_integration_example rokae_adapter)
    endif()
    
    if(ROKAE_MOTION_PLANNER_AVAILABLE)
        target_link_libraries(vision_integration_example rokae_motion_planner)
    endif()
    
    # Windows网络库支持
    if(WIN32)
        target_link_libraries(vision_integration_example ws2_32)
    endif()
    
    ament_target_dependencies(vision_integration_example ${dependencies})
    set(VISION_INTEGRATION_EXAMPLE_AVAILABLE TRUE)
else()
    set(VISION_INTEGRATION_EXAMPLE_AVAILABLE FALSE)
endif()

# ==================== 安装配置 ====================

# 收集所有库目标
set(INSTALL_TARGETS ${PROJECT_NAME}_core)

if(ROKAE_ADAPTER_AVAILABLE)
    list(APPEND INSTALL_TARGETS rokae_adapter)
endif()

if(ROKAE_MOTION_PLANNER_AVAILABLE)
    list(APPEND INSTALL_TARGETS rokae_motion_planner)
endif()

if(VISION_COMMUNICATION_AVAILABLE)
    list(APPEND INSTALL_TARGETS vision_communication)
endif()

if(ELU_ADAPTER_AVAILABLE)
    list(APPEND INSTALL_TARGETS elu_adapter)
endif()

# 安装库和插件
install(TARGETS ${INSTALL_TARGETS}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 收集所有可执行文件目标
set(EXECUTABLE_TARGETS)

if(ROKAE_INTEGRATION_EXAMPLE_AVAILABLE)
    list(APPEND EXECUTABLE_TARGETS rokae_integration_example)
endif()

if(MOTION_CONTROLLER_NODE_AVAILABLE)
    list(APPEND EXECUTABLE_TARGETS motion_controller_node)
endif()

if(STATUS_MONITOR_NODE_AVAILABLE)
    list(APPEND EXECUTABLE_TARGETS status_monitor_node)
endif()

if(VISION_COMMUNICATION_NODE_AVAILABLE)
    list(APPEND EXECUTABLE_TARGETS vision_communication_node)
endif()

if(VISION_INTEGRATION_EXAMPLE_AVAILABLE)
    list(APPEND EXECUTABLE_TARGETS vision_integration_example)
endif()

# 安装可执行文件
if(EXECUTABLE_TARGETS)
    install(TARGETS ${EXECUTABLE_TARGETS}
      DESTINATION lib/${PROJECT_NAME}
    )
endif()

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
if(ELU_ADAPTER_AVAILABLE AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins/elu_adapter/elu_adapter_plugins.xml)
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
    # 核心库测试
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
    if(XCORE_SDK_LIBRARY_FOUND AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_rokae_adapter.cpp)
        ament_add_gtest(test_rokae_adapter test/unit/test_rokae_adapter.cpp)
        target_link_libraries(test_rokae_adapter ${PROJECT_NAME}_core XCoreSDK::XCoreSDK eigen)
        ament_target_dependencies(test_rokae_adapter ${dependencies})
    endif()
    
    # 视觉通信模块测试
    if(VISION_COMMUNICATION_AVAILABLE AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_vision_communication.cpp)
        ament_add_gtest(test_vision_communication 
          test/unit/test_vision_communication.cpp
        )
        target_link_libraries(test_vision_communication 
          ${PROJECT_NAME}_core 
          vision_communication
        )
        if(WIN32)
            target_link_libraries(test_vision_communication ws2_32)
        endif()
        ament_target_dependencies(test_vision_communication ${dependencies})
    endif()
    
    # TCP通信集成测试
    if(VISION_COMMUNICATION_AVAILABLE AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/integration/test_tcp_communication.cpp)
        ament_add_gtest(test_tcp_communication 
          test/integration/test_tcp_communication.cpp
        )
        target_link_libraries(test_tcp_communication 
          ${PROJECT_NAME}_core 
          vision_communication
        )
        if(WIN32)
            target_link_libraries(test_tcp_communication ws2_32)
        endif()
        ament_target_dependencies(test_tcp_communication ${dependencies})
    endif()
  endif()
endif()

# ==================== 构建状态报告 ====================

# 在构建结束时输出状态报告
message(STATUS "")
message(STATUS "=== ELU Robot Arm Framework Build Configuration ===")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "SDK Support:")
message(STATUS "  XCore SDK Found: ${XCORE_SDK_FOUND}")
message(STATUS "  XCore SDK Library Found: ${XCORE_SDK_LIBRARY_FOUND}")
message(STATUS "  xMateModel Available: ${XMATEMODEL_AVAILABLE}")
message(STATUS "")
message(STATUS "Core Libraries:")
message(STATUS "  Rokae Adapter: ${ROKAE_ADAPTER_AVAILABLE}")
message(STATUS "  Rokae Motion Planner: ${ROKAE_MOTION_PLANNER_AVAILABLE}")
message(STATUS "  Vision Communication: ${VISION_COMMUNICATION_AVAILABLE}")
message(STATUS "  ELU Adapter: ${ELU_ADAPTER_AVAILABLE}")
message(STATUS "")
message(STATUS "Executable Nodes:")
message(STATUS "  Rokae Integration Example: ${ROKAE_INTEGRATION_EXAMPLE_AVAILABLE}")
message(STATUS "  Vision Communication Node: ${VISION_COMMUNICATION_NODE_AVAILABLE}")
message(STATUS "  Vision Integration Example: ${VISION_INTEGRATION_EXAMPLE_AVAILABLE}")
message(STATUS "  Motion Controller Node: ${MOTION_CONTROLLER_NODE_AVAILABLE}")
message(STATUS "  Status Monitor Node: ${STATUS_MONITOR_NODE_AVAILABLE}")
message(STATUS "")
message(STATUS "Install Targets: ${INSTALL_TARGETS}")
message(STATUS "Executable Targets: ${EXECUTABLE_TARGETS}")
message(STATUS "")
message(STATUS "Build Options:")
message(STATUS "  XCORE_LINK_SHARED_LIBS: ${XCORE_LINK_SHARED_LIBS}")
message(STATUS "  XCORE_USE_XMATE_MODEL: ${XCORE_USE_XMATE_MODEL}")
message(STATUS "  BUILD_VISION_SUPPORT: ${BUILD_VISION_SUPPORT}")
message(STATUS "  BUILD_TESTING: ${BUILD_TESTING}")
message(STATUS "==================================================")
message(STATUS "")

# 给出构建建议
if(NOT XCORE_SDK_LIBRARY_FOUND)
    message(STATUS "💡 Build Tips:")
    message(STATUS "   To enable full Rokae robot support, place the xCore SDK at:")
    message(STATUS "   ${CMAKE_CURRENT_SOURCE_DIR}/third_party/xcore_sdk/")
    message(STATUS "")
endif()

if(NOT VISION_COMMUNICATION_AVAILABLE AND BUILD_VISION_SUPPORT)
    message(STATUS "💡 Vision Support Tips:")
    message(STATUS "   To enable vision communication, create the source file at:")
    message(STATUS "   ${CMAKE_CURRENT_SOURCE_DIR}/src/vision/vision_communication.cpp")
    message(STATUS "")
endif()

ament_package()