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
