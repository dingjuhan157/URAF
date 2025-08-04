# ELU机械臂运动控制框架

## 概述

ELU机械臂运动控制框架是一个基于ROS2的通用机械臂控制系统，采用分层解耦和插件化的设计理念，支持多种品牌机械臂的统一控制，并集成了完整的TCP视觉通信系统。

## 特性

- 🔌 **插件化架构**: 支持动态加载不同品牌机械臂适配器
- 🛡️ **安全保障**: 内置多层安全检查机制
- 🎯 **统一接口**: 提供标准化的控制API
- 📊 **实时监控**: 完善的状态监控和错误处理
- ⚡ **高性能**: 满足工业级实时控制要求
- 🔧 **易扩展**: 便于添加新的机械臂支持
- 🎯 **视觉集成**: 内置TCP视觉通信模块，支持视觉引导操作
- 🤖 **智能抓取**: 完整的视觉检测→抓取→放置自动化流程

## 新增视觉功能

### 🎯 视觉通信特性
- **TCP客户端连接**: 机器人作为客户端连接视觉服务器
- **双向通信**: 支持位姿查询响应和拍照指令发送
- **自动重连**: 断线自动重连机制和心跳监控
- **多工位支持**: 支持多个检测工位的配置和管理
- **实时结果处理**: 视觉检测结果的实时解析和ROS消息发布

### 📡 支持的通信协议
- **位姿查询响应**: `getRobotPose,` → `p,x,y,z,a,b,c,`
- **拍照指令发送**: `d,x,y,z,a,b,c,stationId,`
- **视觉结果接收**: `code,x,y,z,a,b,c,...` (支持多目标检测)

## 系统要求

- Ubuntu 22.04.5 LTS
- ROS2 Humble
- C++17 编译器
- CMake 3.8+
- YAML-CPP库
- 网络库支持 (Linux自带, Windows需要ws2_32)

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
sudo apt install ros-humble-rqt-graph
sudo apt install ros-humble-rqt-plot
```

### 2. 构建框架

```bash
# 创建工作空间
mkdir -p ~/elu_robot_ws/src
cd ~/elu_robot_ws/src

# 克隆代码（或者直接拷贝创建的包）
# git clone <repository_url>

# 构建（支持多种配置选项）
cd ~/elu_robot_ws

# 完整构建（包含视觉支持）
colcon build --symlink-install

# 仅构建核心功能（不包含视觉）
colcon build --symlink-install --cmake-args -DBUILD_VISION_SUPPORT=OFF

# 调试构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 源环境
source install/setup.bash
```

### 3. 启动框架

```bash
# 启动单机械臂控制（传统模式）
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py

# 启动视觉集成演示（推荐）
ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py

# 启动独立视觉通信节点
ros2 run elu_robot_arm_framework vision_communication_node
```

## 使用指南

### 机械臂配置

编辑 `config/robots/rokae_cr7_config.yaml` 文件：

```yaml
robot_arms:
  rokae_cr7:
    type: "rokae_adapter"
    ip_address: "192.168.0.160"
    local_ip: "192.168.0.100"
    port: 8080
    dof: 6
    max_payload: 7.0
    # ... 其他配置
```

### 视觉系统配置

编辑 `config/vision_config.yaml` 文件：

```yaml
vision_server:
  ip_address: "192.168.1.100"
  port: 8080
  auto_connect: true
  auto_reconnect: true
  reconnect_interval: 5.0

stations:
  station_1:
    id: 1
    name: "抓取工位"
    scan_position:
      joints: [0.0, -0.3, 0.5, 0.0, 1.57, 0.0]
    approach_height: 0.05
```

### 基本运动控制

```bash
# 关节运动
ros2 topic pub /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{robot_id: "rokae_cr7", motion_type: 0, joint_positions: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], speed_ratio: 0.5}'

# 笛卡尔运动
ros2 topic pub /motion_command elu_robot_arm_framework/msg/MotionCommand \
  '{robot_id: "rokae_cr7", motion_type: 1, target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}}, speed_ratio: 0.5}'
```

### 视觉系统操作

```bash
# 连接视觉服务器
ros2 service call /connect_vision std_srvs/srv/Trigger

# 触发视觉检测
ros2 service call /trigger_vision elu_robot_arm_framework/srv/TriggerVision \
  '{station_id: 1, timeout: 15.0}'

# 启动完整演示
ros2 service call /start_vision_demo std_srvs/srv/Trigger

# 监控视觉结果
ros2 topic echo /vision_result

# 监控连接状态
ros2 topic echo /vision_connection_status
```

### 系统监控

```bash
# 查看所有话题
ros2 topic list

# 查看所有服务
ros2 service list

# 监控机器人状态
ros2 topic echo /robot_status

# 监控关节状态
ros2 topic echo /joint_states

# 实时网络拓扑
rqt_graph
```

## 功能模块

### 核心控制模块
- **MotionController**: 运动控制管理器
- **RobotArmInterface**: 机械臂统一接口
- **PluginManager**: 插件管理器
- **SafetyChecker**: 安全检查器

### 视觉通信模块
- **VisionCommunication**: TCP视觉通信核心类
- **VisionIntegrationExample**: 完整视觉集成演示
- **VisionCommunicationNode**: 独立视觉通信节点

### 适配器插件
- **RokaeAdapter**: Rokae机械臂适配器
- **RokaeMotionPlanner**: Rokae运动规划器
- **EluAdapter**: ELU机械臂适配器（可扩展）

## 开发指南

### 添加新机械臂支持

1. 创建适配器类继承 `RobotArmInterface`
```cpp
class NewRobotAdapter : public RobotArmInterface {
public:
    bool connect(const std::string& config_file) override;
    bool moveToJoint(const std::vector<double>& joints, double speed_ratio = 1.0) override;
    // ... 实现所有虚函数
};
```

2. 实现所有虚函数接口
3. 创建插件描述文件
4. 更新CMakeLists.txt
5. 添加配置文件

### 扩展视觉功能

1. **自定义检测结果处理**:
```cpp
void handleCustomVisionResult(const VisionDetectionResult& result) {
    for (const auto& pose : result.poses) {
        // 自定义处理逻辑
    }
}
```

2. **添加新的通信协议**:
```cpp
void processReceivedMessage(const std::string& message) {
    if (message.find("custom_command") == 0) {
        handleCustomCommand(message);
    }
}
```

3. **配置新工位**:
```yaml
stations:
  station_3:
    id: 3
    name: "新检测工位"
    scan_position:
      joints: [0.2, -0.4, 0.6, 0.0, 1.0, 0.2]
```

详细步骤请参考 [开发文档](docs/development_guide.md)

## API文档

### 核心接口

- `RobotArmInterface`: 机械臂统一接口基类
- `MotionController`: 运动控制管理器
- `PluginManager`: 插件管理器
- `SafetyChecker`: 安全检查器
- `VisionCommunication`: 视觉通信接口

### 消息类型

- `MotionCommand`: 运动命令消息
- `RobotStatus`: 机械臂状态消息
- `VisionResult`: 视觉检测结果消息

### 服务类型

- `SwitchRobot`: 切换机械臂服务
- `ExecuteMotion`: 执行运动服务
- `TriggerVision`: 触发视觉检测服务

## 部署方案

### 1. 基础机械臂控制

```bash
# 仅机械臂控制，不包含视觉
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py \
    robot_ip:=192.168.0.160 \
    config_file:=config/robots/rokae_cr7_config.yaml
```

### 2. 完整视觉集成系统

```bash
# 完整系统：机械臂 + 视觉 + 自动演示
ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py \
    vision_server_ip:=192.168.1.100 \
    vision_server_port:=8080 \
    robot_ip:=192.168.0.160 \
    demo_mode:=true \
    use_rviz:=true
```

### 3. 分布式部署

```bash
# 机器人控制节点
ros2 run elu_robot_arm_framework rokae_integration_example

# 独立视觉通信节点（可在不同机器上运行）
ros2 run elu_robot_arm_framework vision_communication_node \
    --ros-args -p vision_server_ip:=192.168.1.100
```

### 4. 开发调试模式

```bash
# 带调试和可视化工具
ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py \
    log_level:=debug \
    use_rviz:=true \
    use_rqt:=true \
    auto_start:=false
```

## 故障排除

### 常见问题

1. **机械臂连接失败**
   ```bash
   # 检查网络连通性
   ping 192.168.0.160
   
   # 检查配置文件
   cat config/robots/rokae_cr7_config.yaml
   ```

2. **视觉服务器连接失败**
   ```bash
   # 检查视觉服务器
   ping 192.168.1.100
   telnet 192.168.1.100 8080
   
   # 查看连接状态
   ros2 topic echo /vision_connection_status
   ```

3. **编译错误**
   ```bash
   # 检查依赖
   rosdep install --from-paths src --ignore-src -r -y
   
   # 清理重建
   colcon build --cmake-clean-cache
   ```

### 日志分析

```bash
# 查看节点日志
ros2 node info /vision_communication
ros2 node info /vision_integration_example

# 实时日志监控
ros2 topic echo /rosout --qos-profile system_default
```

## 性能优化

### 网络优化
- 使用专用网络连接
- 调整TCP缓冲区大小
- 优化消息传输频率

### 运动优化
- 调整速度和加速度参数
- 使用转弯区减少停顿
- 优化路径规划算法

### 系统监控
- CPU和内存使用监控
- 网络延迟测试
- 实时性能分析

## 许可证

Apache License 2.0

## 更新日志

### v2.0.0 - 视觉集成版本
- ✨ 新增TCP视觉通信模块
- ✨ 新增视觉引导抓取演示
- ✨ 新增多工位检测支持
- ✨ 新增自动重连机制
- 🔧 优化CMakeLists.txt构建配置
- 📚 完善文档和使用指南

### v1.0.0 - 基础版本
- 🎉 基础机械臂控制框架
- 🔌 插件化架构设计
- 🛡️ 安全检查机制
- 📊 状态监控系统

## 贡献

欢迎提交Issue和Pull Request！

### 贡献指南
1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开 Pull Request

## 联系方式

- 邮箱: info@elu-ai.com
- 网站: https://www.elu-ai.com


