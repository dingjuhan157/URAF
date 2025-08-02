#!/usr/bin/env python3
"""
启动文件：rokae_integration_demo.launch.py
用于启动Rokae机械臂集成演示
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
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
    
    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode',
        default_value='true',
        description='Enable automatic demo sequence'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically connect to robot on startup'
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
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # 是否启用RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # 是否启用rqt工具
    use_rqt_arg = DeclareLaunchArgument(
        'use_rqt',
        default_value='false',
        description='Launch rqt tools for monitoring'
    )

    # 主要节点：Rokae集成示例
    rokae_integration_node = Node(
        package='elu_robot_arm_framework',
        executable='rokae_integration_example',
        name='rokae_integration_example',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'demo_mode': LaunchConfiguration('demo_mode'),
            'auto_start': LaunchConfiguration('auto_start'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'local_ip': LaunchConfiguration('local_ip'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # Robot State Publisher (用于TF变换)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': get_robot_description(),
            'use_sim_time': False,
        }],
    )

    # Joint State Publisher (如果需要手动控制关节)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('demo_mode'), "' == 'false'"
        ])),
    )

    # RViz2 可视化
    rviz_config_file = os.path.join(
        FindPackageShare('elu_robot_arm_framework').find('elu_robot_arm_framework'),
        'config', 'rviz', 'rokae_robot.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # rqt graph (网络拓扑)
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rqt')),
    )

    # rqt plot (数据绘图)
    rqt_plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        output='screen',
        arguments=['/joint_states/position[0]:position[1]:position[2]:position[3]:position[4]:position[5]'],
        condition=IfCondition(LaunchConfiguration('use_rqt')),
    )

    # 状态监控节点
    status_monitor_node = Node(
        package='elu_robot_arm_framework',
        executable='status_monitor_node',
        name='status_monitor_node',
        output='screen',
        parameters=[{
            'update_rate': 10.0,
            'enable_diagnostics': True,
        }],
    )

    # 启动信息
    startup_info = LogInfo(
        msg=[
            '\n',
            '🤖 ================================\n',
            '   Rokae Robot Integration Demo\n',
            '================================\n',
            'Robot IP: ', LaunchConfiguration('robot_ip'), '\n',
            'Local IP: ', LaunchConfiguration('local_ip'), '\n',
            'Config: ', LaunchConfiguration('config_file'), '\n',
            'Demo Mode: ', LaunchConfiguration('demo_mode'), '\n',
            'Auto Start: ', LaunchConfiguration('auto_start'), '\n',
            'Log Level: ', LaunchConfiguration('log_level'), '\n',
            '================================\n',
            '📋 Available services:\n',
            '  ros2 service call /connect_robot std_srvs/srv/Trigger\n',
            '  ros2 service call /disconnect_robot std_srvs/srv/Trigger\n',
            '  ros2 service call /emergency_stop std_srvs/srv/Trigger\n',
            '📊 Monitor topics:\n',
            '  ros2 topic echo /joint_states\n',
            '  ros2 topic echo /robot_status\n',
            '🔧 Control commands:\n',
            '  ros2 topic pub /motion_command elu_robot_arm_framework/msg/MotionCommand ...\n',
            '================================\n'
        ]
    )

    return LaunchDescription([
        # 启动参数
        config_file_arg,
        demo_mode_arg,
        auto_start_arg,
        robot_ip_arg,
        local_ip_arg,
        log_level_arg,
        use_rviz_arg,
        use_rqt_arg,
        
        # 启动信息
        startup_info,
        
        # 节点
        rokae_integration_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        status_monitor_node,
        
        # 可视化工具
        rviz_node,
        rqt_graph_node,
        rqt_plot_node,
    ])

def get_robot_description():
    """获取机器人URDF描述"""
    pkg_share = FindPackageShare('elu_robot_arm_framework')
    urdf_file = os.path.join(
        pkg_share.find('elu_robot_arm_framework'),
        'urdf', 'rokae_cr7.urdf.xacro'
    )
    
    # 简化的URDF，实际应该从文件读取
    urdf_content = """<?xml version="1.0"?>
<robot name="rokae_cr7" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Joints and Links for 6-DOF arm -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="100" velocity="2.0"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-6.28" upper="6.28" effort="100" velocity="2.0"/>
  </joint>

  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  </joint>

  <link name="link3">
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.25"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-6.28" upper="6.28" effort="50" velocity="3.0"/>
  </joint>

  <link name="link4">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.15"/>
      </geometry>
      <material name="cyan">
        <color rgba="0 1 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-6.28" upper="6.28" effort="30" velocity="3.0"/>
  </joint>

  <link name="link5">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.1"/>
      </geometry>
      <material name="magenta">
        <color rgba="1 0 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="link6"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-6.28" upper="6.28" effort="20" velocity="3.0"/>
  </joint>

  <link name="link6">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- End effector -->
  <joint name="ee_joint" type="fixed">
    <parent link="link6"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

</robot>"""
    
    return urdf_content