主要文件说明
1. RokaeAdapter (rokae_adapter.hpp/cpp)

连接管理: 通过IP地址连接xMateCR7机械臂
运动控制: 提供关节空间和笛卡尔空间运动接口
状态监控: 实时监控机械臂状态、位置和错误信息
安全功能: 急停、错误清除、碰撞检测
参数设置: 速度、加速度、转弯区半径、工具坐标系等

2. RokaeMotionPlanner (rokae_motion_planner.hpp/cpp)

轨迹规划: 关节空间、直线、圆弧轨迹规划
多段轨迹: 支持复杂的多点路径规划
运动学计算: 正逆运动学求解
轨迹验证: 关节限制、碰撞检测、时间优化
SDK集成: 使用Rokae SDK的运动生成器

3. 配置文件 (rokae_cr7_config.yaml)

机械臂连接参数
运动限制和安全参数
工具和工作空间配置
规划器和控制器参数

4. 集成示例 (rokae_integration_example.cpp)

演示完整的使用流程
包含关节运动、笛卡尔运动、轨迹规划、多点路径等示例

主要特性
✅ 已实现的功能

基础连接: 通过配置文件连接机械臂
运动控制: 关节和笛卡尔空间运动
轨迹规划: 基于Rokae SDK的轨迹生成
状态监控: 实时状态和错误监控
安全保护: 急停、限位检查、碰撞检测
参数配置: 速度、加速度、工具坐标系设置

🔧 SDK集成

条件编译支持 (#ifdef XCORE_SDK_AVAILABLE)
即使没有SDK也能编译通过
使用Rokae原生的运动指令和数据类型
集成了Rokae的运动生成器和安全功能

🛡️ 安全特性

关节限制验证
工作空间边界检查
运动超时保护
错误状态监控
急停功能

使用方法

编译项目 (确保CMakeLists.txt已更新)
配置机械臂: 修改 rokae_cr7_config.yaml 中的IP地址等参数
运行示例:

bashros2 run elu_robot_arm_framework rokae_integration_example

在代码中使用:

cpp// 创建适配器
auto adapter = std::make_shared<RokaeAdapter>();

// 连接机械臂
adapter->connect("config/robots/rokae_cr7_config.yaml");

// 执行运动
std::vector<double> joints = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
adapter->moveToJoint(joints);