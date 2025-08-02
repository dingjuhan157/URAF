#!/bin/bash

echo "=========================================="
echo "ROKAE机械臂适配器演示"
echo "=========================================="

# 设置环境
source install/setup.bash

echo "1. 启动ELU框架..."
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py &
LAUNCH_PID=$!

# 等待启动
sleep 5

echo "2. 发送关节运动命令..."
ros2 topic pub --once /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{robot_id: "rokae_xmate6", motion_type: 0, joint_positions: [0.0, 0.5, -0.5, 0.0, 1.0, 0.0], speed_ratio: 0.3}'

sleep 2

echo "3. 发送笛卡尔运动命令..."
ros2 topic pub --once /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{robot_id: "rokae_xmate6", motion_type: 1, target_pose: {position: {x: 0.4, y: 0.2, z: 0.6}}, speed_ratio: 0.2}'

sleep 2

echo "4. 查看机械臂状态..."
timeout 3s ros2 topic echo /robot_status --once || echo "状态查询完成"

echo "5. 停止演示..."
kill $LAUNCH_PID 2>/dev/null || true

echo "ROKAE适配器演示完成！"
