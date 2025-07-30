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
