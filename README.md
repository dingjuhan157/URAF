# ELU机械臂运动控制框架

## 概述

ELU机械臂运动控制框架是一个基于ROS2的通用机械臂控制系统，采用分层解耦和插件化的设计理念，支持多种品牌机械臂的统一控制。

## 特性

- 🔌 **插件化架构**: 支持动态加载不同品牌机械臂适配器
- 🛡️ **安全保障**: 内置多层安全检查机制
- 🎯 **统一接口**: 提供标准化的控制API
- 📊 **实时监控**: 完善的状态监控和错误处理
- ⚡ **高性能**: 满足工业级实时控制要求
- 🔧 **易扩展**: 便于添加新的机械臂支持

## 系统要求

- Ubuntu 22.04.5 LTS
- ROS2 Humble
- C++17 编译器
- CMake 3.8+
- YAML-CPP库

## 快速开始

### 1. 安装依赖

```bash
# 安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装额外依赖
sudo apt install libyaml-cpp-dev
sudo apt install ros-humble-moveit
sudo apt install ros-humble-pluginlib
```

### 2. 构建框架

```bash
# 创建工作空间
mkdir -p ~/elu_robot_ws/src
cd ~/elu_robot_ws/src

# 克隆代码（或者直接拷贝创建的包）
# git clone <repository_url>

# 构建
cd ~/elu_robot_ws
colcon build --symlink-install

# 源环境
source install/setup.bash
```

### 3. 启动框架

```bash
# 启动单机械臂控制
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py

# 启动多机械臂控制
ros2 launch elu_robot_arm_framework multi_arm_control.launch.py
```

## 使用指南

### 配置机械臂

编辑 `config/robots/robot_config.yaml` 文件：

```yaml
robot_arms:
  your_robot:
    plugin_type: "elu_adapter/EluAdapter"
    ip_address: "192.168.1.100"
    port: 8080
    # ... 其他配置
```

### 发送运动命令

```bash
# 关节运动
ros2 topic pub /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{robot_id: "elu_arm_1", motion_type: 0, joint_positions: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], speed_ratio: 0.5}'

# 笛卡尔运动
ros2 topic pub /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{robot_id: "elu_arm_1", motion_type: 1, target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}}, speed_ratio: 0.5}'
```

### 切换机械臂

```bash
ros2 service call /switch_robot elu_robot_arm_framework/srv/SwitchRobot \
  '{robot_id: "elu_arm_2", force_switch: false}'
```

## 开发指南

### 添加新机械臂支持

1. 创建适配器类继承 `RobotArmInterface`
2. 实现所有虚函数接口
3. 创建插件描述文件
4. 更新CMakeLists.txt
5. 添加配置文件

详细步骤请参考 [开发文档](docs/development_guide.md)

## API文档

### 核心接口

- `RobotArmInterface`: 机械臂统一接口基类
- `MotionController`: 运动控制管理器
- `PluginManager`: 插件管理器
- `SafetyChecker`: 安全检查器

### 消息类型

- `MotionCommand`: 运动命令消息
- `RobotStatus`: 机械臂状态消息

### 服务类型

- `SwitchRobot`: 切换机械臂服务
- `ExecuteMotion`: 执行运动服务

## 许可证

Apache License 2.0

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

- 邮箱: dev@elu-robotics.com
- 网站: https://www.elu-robotics.com
