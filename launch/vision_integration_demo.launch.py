#!/usr/bin/env python3
"""
启动文件：vision_integration_demo.launch.py
用于启动机器人视觉集成演示系统
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare('elu_robot_arm_framework')
    
    # 启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=[pkg_share, '/config/robots/rokae_cr7_config.yaml'],
        description='Path to robot configuration file'
    )
    
    vision_server_ip_arg = DeclareLaunchArgument(
        'vision_server_ip',
        default_value='192.168.1.100',
        description='Vision server IP address'
    )
    
    vision_server_port_arg = DeclareLaunchArgument(
        'vision_server_port',
        default_value='8080',
        description='Vision server port'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.0.160',
        description='IP address of the Rokae robot'
    )
    
    local_ip_arg = DeclareLaunchArgument(
        'local_ip',
        default_value='192.168.0.100',
        description='Local IP address for real-time communication'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically connect and start demo'
    )
    
    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode',
        default_value='true',
        description='Enable demonstration mode'
    )
    
    auto_reconnect_arg = DeclareLaunchArgument(
        'auto_reconnect',
        default_value='true',
        description='Enable automatic reconnection to vision server'
    )
    
    reconnect_interval_arg = DeclareLaunchArgument(
        'reconnect_interval',
        default_value='5.0',
        description='Reconnection interval in seconds'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # 可视化工具参数
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    use_rqt_arg = DeclareLaunchArgument(
        'use_rqt',
        default_value='false',
        description='Launch rqt tools for monitoring'
    )

    # 主要节点：视觉通信节点
    vision_communication_node = Node(
        package='elu_robot_arm_framework',
        executable='vision_communication_node',
        name='vision_communication',
        output='screen',
        parameters=[{
            'server_ip': LaunchConfiguration('vision_server_ip'),
            'server_port': LaunchConfiguration('vision_server_port'),
            'auto_reconnect': LaunchConfiguration('auto_reconnect'),
            'reconnect_interval': LaunchConfiguration('reconnect_interval'),
            'auto_connect': LaunchConfiguration('auto_start'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('vision_result', '/vision_result'),
            ('vision_connection_status', '/vision_connection_status'),
            ('trigger_vision', '/trigger_vision'),
            ('connect_vision', '/connect_vision'),
            ('disconnect_vision', '/disconnect_vision'),
        ]
    )

    # 视觉集成示例节点
    vision_integration_node = Node(
        package='elu_robot_arm_framework',
        executable='vision_integration_example',
        name='vision_integration_example',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'vision_server_ip': LaunchConfiguration('vision_server_ip'),
            'vision_server_port': LaunchConfiguration('vision_server_port'),
            'auto_start': LaunchConfiguration('auto_start'),
            'demo_mode': LaunchConfiguration('demo_mode'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'local_ip': LaunchConfiguration('local_ip'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # 状态监控节点
    status_monitor_node = Node(
        package='elu_robot_arm_framework',
        executable='status_monitor_node',
        name='status_monitor_node',
        output='screen',
        parameters=[{
            'update_rate': 2.0,
            'enable_diagnostics': True,
            'monitor_vision': True,
        }],
    )

    # RViz可视化
    rviz_config_file = os.path.join(
        FindPackageShare('elu_robot_arm_framework').find('elu_robot_arm_framework'),
        'config', 'rviz', 'vision_integration.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # rqt工具组合
    rqt_tools_group = GroupAction([
        # rqt graph (网络拓扑)
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
            output='screen',
        ),
        
        # rqt plot (数据绘图)
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            output='screen',
            arguments=['/vision_result/code', '/vision_connection_status'],
        ),
        
        # rqt console (日志查看)
        Node(
            package='rqt_console',
            executable='rqt_console',
            name='rqt_console',
            output='screen',
        ),
    ], condition=IfCondition(LaunchConfiguration('use_rqt')))

    # 启动信息
    startup_info = LogInfo(
        msg=[
            '\n',
            '🤖🎯 =========================================\n',
            '   Robot Vision Integration Demo\n',
            '=========================================\n',
            'Robot Config: ', LaunchConfiguration('config_file'), '\n',
            'Robot IP: ', LaunchConfiguration('robot_ip'), '\n',
            'Local IP: ', LaunchConfiguration('local_ip'), '\n',
            'Vision Server: ', LaunchConfiguration('vision_server_ip'), ':', LaunchConfiguration('vision_server_port'), '\n',
            'Auto Start: ', LaunchConfiguration('auto_start'), '\n',
            'Demo Mode: ', LaunchConfiguration('demo_mode'), '\n',
            'Auto Reconnect: ', LaunchConfiguration('auto_reconnect'), '\n',
            'Log Level: ', LaunchConfiguration('log_level'), '\n',
            '=========================================\n',
            '📋 Available services:\n',
            '  # Robot control\n',
            '  ros2 service call /connect_robot std_srvs/srv/Trigger\n',
            '  ros2 service call /disconnect_robot std_srvs/srv/Trigger\n',
            '  ros2 service call /emergency_stop std_srvs/srv/Trigger\n',
            '  # Vision system\n',
            '  ros2 service call /connect_vision std_srvs/srv/Trigger\n',
            '  ros2 service call /disconnect_vision std_srvs/srv/Trigger\n',
            '  ros2 service call /trigger_vision elu_robot_arm_framework/srv/TriggerVision "{station_id: 1, timeout: 10.0}"\n',
            '  # Demo control\n',
            '  ros2 service call /start_vision_demo std_srvs/srv/Trigger\n',
            '📊 Monitor topics:\n',
            '  ros2 topic echo /vision_result\n',
            '  ros2 topic echo /vision_connection_status\n',
            '  ros2 topic echo /joint_states\n',
            '  ros2 topic echo /robot_status\n',
            '🔧 Testing commands:\n',
            '  # Test vision detection\n',
            '  ros2 service call /trigger_vision elu_robot_arm_framework/srv/TriggerVision "{station_id: 1, timeout: 15.0}"\n',
            '  # Monitor vision results\n',
            '  ros2 topic echo /vision_result --once\n',
            '  # Check connection status\n',
            '  ros2 topic echo /vision_connection_status --once\n',
            '📡 TCP Communication Protocol:\n',
            '  - Robot -> Vision: "d,x,y,z,a,b,c,stationId," (trigger capture)\n',
            '  - Vision -> Robot: "getRobotPose," (pose query)\n',
            '  - Robot -> Vision: "p,x,y,z,a,b,c," (pose response)\n',
            '  - Vision -> Robot: "code,x,y,z,a,b,c,..." (detection result)\n',
            '=========================================\n'
        ]
    )

    return LaunchDescription([
        # 声明所有参数
        config_file_arg,
        vision_server_ip_arg,
        vision_server_port_arg,
        robot_ip_arg,
        local_ip_arg,
        auto_start_arg,
        demo_mode_arg,
        auto_reconnect_arg,
        reconnect_interval_arg,
        log_level_arg,
        use_rviz_arg,
        use_rqt_arg,
        
        # 显示启动信息
        startup_info,
        
        # 启动核心节点
        vision_communication_node,
        vision_integration_node,
        status_monitor_node,
        
        # 可视化工具
        rviz_node,
        rqt_tools_group,
    ])

# 使用示例:
#
# 1. 基本启动 (自动连接和演示):
#    ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py
#
# 2. 自定义参数启动:
#    ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py \
#        vision_server_ip:=192.168.1.200 \
#        vision_server_port:=9090 \
#        robot_ip:=192.168.0.161 \
#        demo_mode:=false \
#        use_rviz:=true
#
# 3. 仅连接不演示:
#    ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py \
#        auto_start:=true \
#        demo_mode:=false
#
# 4. 开发模式 (带可视化工具):
#    ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py \
#        use_rviz:=true \
#        use_rqt:=true \
#        log_level:=debug
#
# 5. 手动控制模式:
#    ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py \
#        auto_start:=false \
#        demo_mode:=false
#    
#    然后手动调用服务:
#    ros2 service call /connect_vision std_srvs/srv/Trigger
#    ros2 service call /start_vision_demo std_srvs/srv/Trigger