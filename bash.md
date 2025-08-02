# 编译项目
cd ~/rokae_ws
colcon build --packages-select elu_robot_arm_framework

# 加载环境
source install/setup.bash

# 运行节点
ros2 run elu_robot_arm_framework rokae_integration_example

# 或使用启动文件
ros2 launch elu_robot_arm_framework rokae_integration_demo.launch.py
