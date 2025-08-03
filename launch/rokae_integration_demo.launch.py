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
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    # 是否启用rqt工具
    use_rqt_arg = DeclareLaunchArgument(
        'use_rqt',
        default_value='false',
        description='Launch rqt tools for monitoring'
    )

    # 是否启用URDF可视化
    use_urdf_arg = DeclareLaunchArgument(
        'use_urdf',
        default_value='false',
        description='Launch robot state publisher with URDF (for future implementation)'
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

    # TODO: 未来添加URDF支持时启用
    # Robot State Publisher (用于TF变换) - 当前已禁用，等待URDF文件完成
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': get_robot_description(),
    #         'use_sim_time': False,
    #     }],
    #     condition=IfCondition(LaunchConfiguration('use_urdf')),
    # )

    # Joint State Publisher (如果需要手动控制关节) - 当前已禁用
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen',
    #     condition=IfCondition(PythonExpression([
    #         "'", LaunchConfiguration('demo_mode'), "' == 'false' and '", 
    #         LaunchConfiguration('use_urdf'), "' == 'true'"
    #     ])),
    # )

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
            'URDF Enabled: ', LaunchConfiguration('use_urdf'), '\n',
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
            '📝 Note: URDF visualization currently disabled\n',
            '  Enable with: use_urdf:=true (when URDF files are ready)\n',
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
        use_urdf_arg,  # 保留接口参数
        
        # 启动信息
        startup_info,
        
        # 节点
        rokae_integration_node,
        # robot_state_publisher_node,  # 已注释，等待URDF文件
        # joint_state_publisher_node,  # 已注释，等待URDF文件
        status_monitor_node,
        
        # 可视化工具
        rviz_node,
        rqt_graph_node,
        rqt_plot_node,
    ])

def get_robot_description():
    """
    获取机器人URDF描述
    
    TODO: 实现URDF文件加载
    当URDF文件准备好时，可以按以下方式实现：
    
    1. 从文件加载URDF:
       pkg_share = FindPackageShare('elu_robot_arm_framework')
       urdf_file = os.path.join(
           pkg_share.find('elu_robot_arm_framework'),
           'urdf', 'rokae_cr7.urdf.xacro'
       )
       with open(urdf_file, 'r') as f:
           return f.read()
    
    2. 处理xacro文件:
       from launch.substitutions import Command
       return Command(['xacro ', urdf_file])
    
    Returns:
        str: 机器人URDF描述（当前为空，等待实现）
    """
    # 当前返回空字符串，等待URDF文件准备完成
    return ""

def load_urdf_from_file(urdf_file_path):
    """
    从文件加载URDF描述
    
    Args:
        urdf_file_path (str): URDF文件路径
        
    Returns:
        str: URDF内容
        
    TODO: 实现URDF文件验证和错误处理
    """
    try:
        with open(urdf_file_path, 'r') as f:
            return f.read()
    except FileNotFoundError:
        print(f"URDF文件未找到: {urdf_file_path}")
        return ""
    except Exception as e:
        print(f"加载URDF文件时出错: {e}")
        return ""

def process_xacro_file(xacro_file_path):
    """
    处理xacro文件生成URDF
    
    Args:
        xacro_file_path (str): xacro文件路径
        
    Returns:
        Command: 用于处理xacro的launch命令
        
    TODO: 实现xacro参数传递和错误处理
    """
    from launch.substitutions import Command
    return Command(['xacro ', xacro_file_path])

# 启用URDF时的使用示例:
# 
# 启动时设置参数:
# ros2 launch elu_robot_arm_framework rokae_integration_demo.launch.py \
#     use_urdf:=true \
#     use_rviz:=true \
#     demo_mode:=false
#
# 这将启动完整的可视化环境包括:
# - Robot State Publisher (发布TF变换)
# - Joint State Publisher GUI (手动关节控制)
# - RViz (3D可视化)
# - 机械臂控制节点