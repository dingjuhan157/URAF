# 机械臂运动控制框架设计文档

## 项目概述

本文档描述了一个适配不同机械臂的ROS2运动控制框架的设计思路和实现流程。该框架采用分层解耦的设计原则，通过插件化的方式支持多种机械臂设备，提供统一的控制接口，并集成了完整的TCP视觉通信系统，实现视觉引导的智能抓取操作。

## 整体架构设计

### 核心设计原则

- **分层解耦**：硬件抽象层 → 统一接口层 → 应用层
- **插件化**：不同机械臂作为可插拔模块
- **标准化**：统一的消息格式和服务接口
- **可扩展性**：便于新增机械臂支持
- **视觉集成**：内置TCP通信的视觉系统集成
- **智能化**：支持视觉引导的自动化操作

### 架构分层

```
应用层 (Application Layer)
├── 视觉集成演示节点
├── 路径规划节点
├── 任务调度节点  
└── 用户界面

业务逻辑层 (Business Logic Layer)
├── 视觉引导抓取逻辑
├── 多工位检测管理
├── 自动化工作流程
└── 异常处理机制

统一接口层 (Unified Interface Layer)
├── 运动控制抽象接口
├── 视觉通信接口
├── 状态监控接口
└── 配置管理接口

硬件适配层 (Hardware Adaptation Layer)  
├── Rokae适配器
├── 视觉通信适配器
├── Jaka适配器
├── RealMan适配器
└── 其他厂商适配器

硬件驱动层 (Hardware Driver Layer)
├── 各厂商SDK
├── TCP/IP网络通信
├── 视觉系统接口
└── 通信协议实现

网络通信层 (Network Communication Layer)
├── TCP客户端连接管理
├── 自动重连机制
├── 心跳监控
└── 数据格式转换
```

## 关键组件设计

### 1. 抽象基类设计

定义统一的机械臂接口，所有适配器都需要实现该接口：

```cpp
class RobotArmInterface {
public:
    // 基础连接接口
    virtual bool connect(const std::string& config_file) = 0;
    virtual bool disconnect() = 0;
    virtual bool isConnected() const = 0;

    // 运动控制接口
    virtual bool moveToJoint(const std::vector<double>& joints, double speed_ratio = 1.0) = 0;
    virtual bool moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) = 0;
    virtual bool linearMove(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) = 0;

    // 状态读取接口
    virtual std::vector<double> getCurrentJoints() = 0;
    virtual geometry_msgs::msg::Pose getCurrentPose() = 0;
    virtual RobotState getStatus() = 0;
    virtual std::string getErrorMessage() = 0;

    // 控制参数设置
    virtual bool setSpeed(double speed_ratio) = 0;
    virtual bool setAcceleration(double acceleration_ratio) = 0;
    virtual bool setPayload(double payload_kg) = 0;

    // 安全控制
    virtual bool emergencyStop() = 0;
    virtual bool clearError() = 0;
    virtual bool enable() = 0;
    virtual bool disable() = 0;

    // 信息获取
    virtual std::string getRobotModel() const = 0;
    virtual int getDoF() const = 0;
    virtual double getMaxPayload() const = 0;
    virtual std::vector<double> getJointLimits() const = 0;
};
```

### 2. 视觉通信接口设计

新增视觉通信核心类，处理与视觉系统的TCP通信：

```cpp
class VisionCommunication : public rclcpp::Node {
public:
    VisionCommunication(std::shared_ptr<RobotArmInterface> robot_adapter);
    
    // 连接管理
    bool connectToVisionServer(const std::string& server_ip, int server_port);
    void disconnect();
    bool isConnected() const;
    TcpConnectionState getConnectionState() const;
    
    // 视觉检测
    VisionDetectionResult triggerCapture(int station_id, double timeout_sec = 10.0);
    bool triggerCaptureAsync(int station_id);
    VisionDetectionResult getLatestResult();
    
    // 配置管理
    void setAutoReconnect(bool enable, double interval_sec = 5.0);

private:
    // TCP通信核心
    bool createSocket();
    bool connectSocket(const std::string& server_ip, int server_port);
    bool sendData(const std::string& data);
    int receiveData(char* buffer, int max_size);
    
    // 协议处理
    void processReceivedMessage(const std::string& message);
    std::string handlePoseQuery();
    void handleVisionResult(const std::string& message);
    bool sendCaptureCommand(int station_id);
    
    // 数据转换
    std::string poseToString(const geometry_msgs::msg::Pose& pose);
    geometry_msgs::msg::Pose stringToPose(const std::string& pose_str);
};
```

### 3. 配置管理系统

扩展配置文件以支持视觉系统：

```yaml
# robot_config.yaml
robot_arms:
  rokae_cr7:
    type: "rokae_adapter"
    ip: "192.168.0.160"
    local_ip: "192.168.0.100"
    dof: 6
    max_payload: 7.0
    workspace_limits: [...]
    
# vision_config.yaml
vision_server:
  ip_address: "192.168.1.100"
  port: 8080
  connection_timeout: 5000
  auto_reconnect: true
  reconnect_interval: 5.0
  heartbeat_interval: 30.0

communication:
  message_delimiter: ","
  message_terminator: "\n"
  coordinate_precision: 3
  response_timeout: 15.0

stations:
  station_1:
    id: 1
    name: "抓取工位"
    scan_position:
      joints: [0.0, -0.3, 0.5, 0.0, 1.57, 0.0]
    approach_height: 0.05
    place_offset: [0.2, 0.0, 0.0]
```

### 4. 运动控制管理器

扩展核心控制组件以支持视觉引导操作：

```cpp
class MotionController {
    std::shared_ptr<RobotArmInterface> current_arm_;
    std::shared_ptr<VisionCommunication> vision_comm_;
    TrajectoryPlanner planner_;
    SafetyChecker safety_;
    
public:
    bool executeMotion(const MotionCommand& cmd);
    bool switchRobot(const std::string& robot_id);
    bool emergencyStop();
    RobotStatus getRobotStatus();
    
    // 视觉引导功能
    bool executeVisionGuidedPick(int station_id);
    bool moveToScanPosition(int station_id);
    VisionDetectionResult triggerVisionDetection(int station_id);
    bool executePickAndPlace(const geometry_msgs::msg::Pose& target_pose);
};
```

### 5. 消息和服务定义

新增视觉相关的消息和服务：

**VisionResult.msg**
```
std_msgs/Header header
int32 code                              # 结果代码
string error_message                    # 错误信息
geometry_msgs/Pose[] poses              # 检测到的位姿列表
```

**TriggerVision.srv**
```
int32 station_id                        # 工位号
float64 timeout                         # 超时时间
---
bool success                           # 是否成功
string message                         # 响应消息
int32 result_code                      # 结果代码
geometry_msgs/Pose[] poses             # 检测到的位姿列表
```

## TCP通信协议设计

### 协议格式

1. **位姿查询响应** (被动)
   - 接收: `getRobotPose,`
   - 响应: `p,x,y,z,a,b,c,`

2. **拍照指令发送** (主动)
   - 发送: `d,x,y,z,a,b,c,stationId,`

3. **视觉结果接收** (被动)
   - 接收: `code,x,y,z,a,b,c,...`

### 数据格式规范

- 坐标单位：毫米 (mm)
- 角度单位：度 (°)
- 精度：保留3位小数
- 分隔符：逗号 `,`
- 终止符：换行符 `\n`

### 错误代码定义

```cpp
enum VisionResultCode {
    EMPTY_BOX = 0,              // 空箱
    POINT_CLOUD_NO_COORDS = -1, // 有点云但无坐标
    INVALID_COMMAND = -2,       // 非法指令
    DETECTION_NOT_ENABLED = -3, // 未开启检测模式
    // 正数表示检测到的目标数量
};
```

## 实现流程

### 阶段一：框架搭建 (1-2周)

#### 1. 创建ROS2包结构

```
elu_robot_arm_framework/
├── include/elu_robot_arm_framework/
│   ├── interfaces/
│   │   └── robot_arm_interface.hpp
│   ├── controllers/
│   │   └── motion_controller.hpp
│   ├── adapters/
│   │   └── rokae_adapter.hpp
│   ├── planners/
│   │   └── rokae_motion_planner.hpp
│   ├── utils/
│   │   └── plugin_manager.hpp
│   ├── safety/
│   │   └── safety_checker.hpp
│   └── vision_communication.hpp
├── src/
│   ├── controllers/
│   │   └── motion_controller.cpp
│   ├── adapters/
│   │   └── rokae_adapter.cpp
│   ├── planners/
│   │   └── rokae_motion_planner.cpp
│   ├── utils/
│   │   └── plugin_manager.cpp
│   ├── safety/
│   │   └── safety_checker.cpp
│   ├── vision/
│   │   └── vision_communication.cpp
│   └── nodes/
│       ├── rokae_integration_example.cpp
│       ├── vision_communication_node.cpp
│       ├── vision_integration_example.cpp
│       ├── motion_controller_node.cpp
│       └── status_monitor_node.cpp
├── plugins/
│   ├── elu_adapter/
│   ├── rokae_adapter/
│   └── ur_adapter/
├── config/
│   ├── robots/
│   │   └── rokae_cr7_config.yaml
│   ├── vision_config.yaml
│   └── safety_config.yaml
├── launch/
│   ├── elu_robot_framework.launch.py
│   ├── rokae_integration_demo.launch.py
│   ├── vision_integration_demo.launch.py
│   └── multi_arm_control.launch.py
├── msg/
│   ├── MotionCommand.msg
│   ├── RobotStatus.msg
│   └── VisionResult.msg
├── srv/
│   ├── SwitchRobot.srv
│   ├── ExecuteMotion.srv
│   └── TriggerVision.srv
├── CMakeLists.txt
└── package.xml
```

#### 2. 定义统一消息格式

**MotionCommand.msg**
```
std_msgs/Header header
string robot_id
int32 motion_type  # JOINT_MOVE, CARTESIAN_MOVE, LINEAR_MOVE
float64[] joint_positions
geometry_msgs/Pose target_pose
float64 speed_ratio
bool wait_for_completion
```

**RobotStatus.msg**
```
std_msgs/Header header
string robot_id
int32 status  # IDLE, MOVING, ERROR, etc.
float64[] current_joints
geometry_msgs/Pose current_pose
string error_message
bool is_connected
```

**VisionResult.msg**
```
std_msgs/Header header
int32 code                              # 结果代码
string error_message                    # 错误信息
geometry_msgs/Pose[] poses              # 检测到的位姿列表
```

#### 3. 实现抽象接口基类

建立插件加载机制和基础接口定义，包括视觉通信接口。

### 阶段二：核心功能开发 (2-3周)

#### 1. 运动控制管理器实现

实现统一的运动控制逻辑，包括轨迹规划、执行监控等功能。

#### 2. TCP视觉通信模块

- 实现TCP客户端连接管理
- 处理双向通信协议
- 实现自动重连和心跳监控
- 数据格式转换和验证

#### 3. 路径规划集成

- 集成自定义运动规划器
- 支持不同机械臂的运动学计算
- 实现碰撞检测和路径优化
- 视觉引导路径规划

#### 4. 安全监控系统

- 工作空间边界检查
- 速度和加速度限制
- 碰撞检测
- 紧急停止机制
- 视觉检测结果验证

### 阶段三：设备适配器开发 (3-4周)

#### 1. 实现Rokae适配器

**RokaeAdapter实现**
```cpp
class RokaeAdapter : public RobotArmInterface {
private:
#ifdef XCORE_SDK_AVAILABLE
    std::unique_ptr<rokae::xMateRobot> robot_;
#endif
    std::string robot_ip_;
    std::string local_ip_;
    std::atomic<bool> is_connected_;
    std::atomic<bool> is_moving_;
    
public:
    RokaeAdapter();
    
    bool connect(const std::string& config_file) override;
    bool disconnect() override;
    bool moveToJoint(const std::vector<double>& joints, double speed_ratio = 1.0) override;
    bool moveToPose(const geometry_msgs::msg::Pose& pose, double speed_ratio = 1.0) override;
    std::vector<double> getCurrentJoints() override;
    geometry_msgs::msg::Pose getCurrentPose() override;
    RobotState getStatus() override;
    
    // 扩展接口
    bool moveToJointAsync(const std::vector<double>& joints);
    bool setBlendRadius(double radius);
};
```

#### 2. 实现视觉通信适配器

**VisionCommunication实现**
```cpp
class VisionCommunication : public rclcpp::Node {
private:
    std::shared_ptr<RobotArmInterface> robot_adapter_;
    
    // 网络相关
#ifdef _WIN32
    SOCKET socket_fd_;
    WSADATA wsa_data_;
#else
    int socket_fd_;
#endif
    std::string server_ip_;
    int server_port_;
    std::atomic<TcpConnectionState> connection_state_;
    
    // 线程管理
    std::thread comm_thread_;
    std::thread reconnect_thread_;
    std::atomic<bool> thread_running_;
    
    // 结果管理
    std::queue<VisionDetectionResult> result_queue_;
    std::mutex result_mutex_;
    std::condition_variable result_condition_;
    
public:
    VisionCommunication(std::shared_ptr<RobotArmInterface> robot_adapter);
    
    bool connectToVisionServer(const std::string& server_ip, int server_port);
    VisionDetectionResult triggerCapture(int station_id, double timeout_sec = 10.0);
    
private:
    void communicationThread();
    void processReceivedMessage(const std::string& message);
    std::string handlePoseQuery();
    void handleVisionResult(const std::string& message);
};
```

#### 3. 通信层实现

- TCP/IP通信协议实现
- 跨平台网络库支持 (Windows/Linux)
- 实时控制数据传输
- 错误处理和重连机制
- 数据包验证和校验

### 阶段四：视觉集成开发 (2-3周)

#### 1. 视觉引导演示节点

```cpp
class VisionIntegrationExample : public rclcpp::Node {
private:
    std::shared_ptr<RokaeAdapter> robot_adapter_;
    std::shared_ptr<VisionCommunication> vision_comm_;
    
public:
    VisionIntegrationExample();
    
private:
    void startVisionDemo();
    void demoMoveToScanPosition();
    void demoTriggerVisionDetection();
    void demoProcessVisionResults();
    void demoPickAndPlace();
    void demoReturnHome();
};
```

#### 2. 多工位管理

- 工位配置管理
- 扫描位置设定
- 抓取策略选择
- 放置位置计算

#### 3. 自动化工作流程

- 视觉检测 → 运动规划 → 抓取执行 → 放置完成
- 异常处理和恢复机制
- 多目标处理策略
- 循环作业支持

### 阶段五：集成测试与优化 (2-3周)

#### 1. 单元测试

为每个组件编写单元测试，确保功能正确性：

```cpp
// 测试视觉通信
TEST(VisionCommunicationTest, TcpConnection) {
    auto vision_comm = std::make_shared<VisionCommunication>(nullptr);
    EXPECT_TRUE(vision_comm->connectToVisionServer("127.0.0.1", 8080));
}

// 测试Rokae适配器
TEST(RokaeAdapterTest, BasicMovement) {
    RokaeAdapter adapter;
    EXPECT_TRUE(adapter.connect("test_config.yaml"));
    
    std::vector<double> joints = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    EXPECT_TRUE(adapter.moveToJoint(joints));
}
```

#### 2. 集成测试

测试多组件协同和完整工作流程：

- 机械臂与视觉系统协同测试
- 多工位检测场景测试
- 异常处理和恢复测试
- 性能压力测试

#### 3. 性能优化

- 通信延迟优化
- 内存使用优化
- 实时性能调优
- 并发处理优化

#### 4. 文档编写

编写用户手册、API文档和维护指南。

## 关键技术实现

### 1. 插件化架构实现

使用ROS2 pluginlib实现动态加载：

**plugin.xml**
```xml
<library path="lib/librokae_adapter">
  <class name="rokae_adapter/RokaeAdapter" type="RokaeAdapter" 
         base_class_type="RobotArmInterface">
    <description>Rokae Robot Arm Adapter</description>
  </class>
</library>
```

**插件管理器**
```cpp
class PluginManager {
private:
    pluginlib::ClassLoader<RobotArmInterface> plugin_loader_;
    std::map<std::string, std::shared_ptr<RobotArmInterface>> loaded_robots_;
    
public:
    bool loadRobot(const std::string& robot_id, const std::string& plugin_name);
    std::shared_ptr<RobotArmInterface> getRobot(const std::string& robot_id);
    bool unloadRobot(const std::string& robot_id);
};
```

### 2. 参数服务器配置

利用ROS2参数系统进行配置管理：

```cpp
class ConfigManager {
public:
    static RobotConfig loadRobotConfig(const std::string& robot_id) {
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("config_loader");
        RobotConfig config;
        
        std::string param_prefix = "robot_arms." + robot_id + ".";
        node->declare_parameter(param_prefix + "type", "");
        node->declare_parameter(param_prefix + "ip", "");
        
        config.type = node->get_parameter(param_prefix + "type").as_string();
        config.ip = node->get_parameter(param_prefix + "ip").as_string();
        
        return config;
    }
};
```

### 3. 状态机设计

为每个机械臂实现状态机：

```cpp
enum class RobotState {
    DISCONNECTED,
    CONNECTING,
    IDLE,
    MOVING,
    ERROR,
    EMERGENCY_STOP
};

enum class TcpConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR
};

class RobotStateMachine {
private:
    RobotState current_state_;
    std::map<RobotState, std::vector<RobotState>> valid_transitions_;
    
public:
    bool transitionTo(RobotState new_state);
    RobotState getCurrentState() const;
    bool isValidTransition(RobotState from, RobotState to) const;
};
```

### 4. 网络通信实现

跨平台TCP通信实现：

```cpp
class TcpClient {
private:
#ifdef _WIN32
    SOCKET socket_fd_;
    WSADATA wsa_data_;
#else
    int socket_fd_;
#endif

public:
    bool initializeNetwork();
    bool createSocket();
    bool connectToServer(const std::string& ip, int port);
    bool sendData(const std::string& data);
    int receiveData(char* buffer, int max_size);
    void closeSocket();
    void cleanupNetwork();
};
```

### 5. 实时性保证

- 使用ROS2实时扩展
- 优化通信频率 (建议100Hz-1000Hz)
- 实现优先级调度
- 内存预分配避免动态分配

## 使用示例

### 启动完整系统

```bash
# 启动视觉集成演示
ros2 launch elu_robot_arm_framework vision_integration_demo.launch.py \
    vision_server_ip:=192.168.1.100 \
    vision_server_port:=8080 \
    robot_ip:=192.168.0.160 \
    demo_mode:=true
```

### 编程接口使用

```cpp
// 初始化系统
auto robot_adapter = std::make_shared<RokaeAdapter>();
auto vision_comm = std::make_shared<VisionCommunication>(robot_adapter);

// 连接设备
robot_adapter->connect("config/robots/rokae_cr7_config.yaml");
vision_comm->connectToVisionServer("192.168.1.100", 8080);

// 执行视觉引导抓取
// 1. 移动到扫描位置
std::vector<double> scan_joints = {0.0, -0.3, 0.5, 0.0, 1.57, 0.0};
robot_adapter->moveToJoint(scan_joints);

// 2. 触发视觉检测
auto result = vision_comm->triggerCapture(1, 15.0);

// 3. 处理检测结果
if (result.code > 0 && !result.poses.empty()) {
    auto target_pose = result.poses[0];
    
    // 4. 执行抓取
    geometry_msgs::msg::Pose pre_pick_pose = target_pose;
    pre_pick_pose.position.z += 0.05;  // 预抓取位置
    
    robot_adapter->moveToPose(pre_pick_pose);
    robot_adapter->linearMove(target_pose);
    
    // 5. 模拟夹爪关闭和提升
    robot_adapter->linearMove(pre_pick_pose);
    
    // 6. 移动到放置位置
    geometry_msgs::msg::Pose place_pose = target_pose;
    place_pose.position.x += 0.2;  // 右移200mm
    robot_adapter->moveToPose(place_pose);
}
```

### 服务调用示例

```bash
# 连接视觉服务器
ros2 service call /connect_vision std_srvs/srv/Trigger

# 触发视觉检测
ros2 service call /trigger_vision elu_robot_arm_framework/srv/TriggerVision \
    '{station_id: 1, timeout: 15.0}'

# 启动自动演示
ros2 service call /start_vision_demo std_srvs/srv/Trigger

# 紧急停止
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

### 话题订阅示例

```bash
# 监控视觉检测结果
ros2 topic echo /vision_result

# 监控机器人状态
ros2 topic echo /robot_status

# 监控连接状态
ros2 topic echo /vision_connection_status

# 监控关节状态
ros2 topic echo /joint_states
```

## 扩展指南

### 添加新机械臂支持

1. 创建新的适配器类继承 `RobotArmInterface`
2. 实现所有虚函数接口
3. 编写plugin.xml配置文件
4. 添加到CMakeLists.txt编译配置
5. 更新机械臂配置文件

### 扩展视觉功能

1. **添加新的检测类型**:
```cpp
enum class DetectionType {
    OBJECT_DETECTION,
    BARCODE_READING,
    QUALITY_INSPECTION,
    DIMENSION_MEASUREMENT
};
```

2. **自定义结果处理**:
```cpp
class CustomVisionProcessor {
public:
    virtual void processResult(const VisionDetectionResult& result) = 0;
    virtual bool validateResult(const VisionDetectionResult& result) = 0;
};
```

3. **多相机支持**:
```yaml
vision_cameras:
  camera_1:
    ip: "192.168.1.100"
    port: 8080
    station_ids: [1, 2]
  camera_2:
    ip: "192.168.1.101"
    port: 8080
    station_ids: [3, 4]
```

### 自定义运动规划算法

框架支持集成自定义的路径规划算法：

```cpp
class CustomTrajectoryPlanner : public TrajectoryPlannerInterface {
public:
    bool planJointTrajectory(const std::vector<double>& start_joints,
                            const std::vector<double>& target_joints,
                            double speed_ratio,
                            trajectory_msgs::msg::JointTrajectory& trajectory) override;
};
```

## 性能指标和优化

### 关键性能指标

- **通信延迟**: < 10ms (局域网环境)
- **运动响应时间**: < 50ms
- **视觉检测超时**: 可配置 (默认15秒)
- **重连间隔**: 可配置 (默认5秒)
- **控制频率**: 100Hz-1000Hz

### 优化策略

1. **网络优化**:
   - 专用网络连接
   - TCP_NODELAY设置
   - 缓冲区大小调优

2. **内存优化**:
   - 对象池模式
   - 预分配缓冲区
   - 智能指针使用

3. **并发优化**:
   - 线程池管理
   - 异步处理
   - 锁优化

## 安全考虑

### 网络安全
- TCP连接加密选项
- 身份验证机制
- 防火墙配置建议

### 机械臂安全
- 工作空间限制
- 速度和加速度限制
- 碰撞检测
- 紧急停止机制

### 数据验证
- 坐标范围检查
- 视觉结果验证
- 通信数据校验

## 总结

该ROS2机械臂运动控制框架具有以下优势：

- **模块化设计**：便于维护和扩展
- **设备无关性**：上层应用无需关心具体硬件
- **插件化架构**：支持运行时动态加载设备
- **统一接口**：简化多设备编程复杂度
- **安全保障**：内置多层安全检查机制
- **实时性能**：满足工业级实时控制要求
- **视觉集成**：完整的TCP视觉通信系统
- **智能化**：支持视觉引导的自动化操作
- **跨平台**：支持Windows和Linux系统

### 技术特色

1. **完整的视觉集成**：从TCP通信到自动化抓取的端到端解决方案
2. **智能重连机制**：网络中断自动恢复，提高系统可靠性
3. **多工位支持**：灵活的工位配置和管理系统
4. **实时监控**：全方位的状态监控和诊断功能
5. **模块化部署**：支持分布式部署和独立运行