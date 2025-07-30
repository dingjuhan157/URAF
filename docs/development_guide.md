# 开发指南

## 添加新机械臂适配器

### 1. 创建适配器类

继承 `RobotArmInterface` 基类：

```cpp
class YourRobotAdapter : public elu_robot_arm_framework::RobotArmInterface
{
public:
  YourRobotAdapter();
  virtual ~YourRobotAdapter();

  // 实现所有纯虚函数
  bool connect(const std::string& config_file) override;
  // ... 其他接口实现
};
```

### 2. 实现SDK集成

集成厂商提供的SDK：

```cpp
bool YourRobotAdapter::connect(const std::string& config_file)
{
  // 1. 读取配置文件
  YAML::Node config = YAML::LoadFile(config_file);
  
  // 2. 初始化SDK
  your_sdk_->initialize();
  
  // 3. 建立连接
  return your_sdk_->connect(ip_address_, port_);
}
```

### 3. 注册插件

使用PLUGINLIB_EXPORT_CLASS宏注册插件：

```cpp
PLUGINLIB_EXPORT_CLASS(your_namespace::YourRobotAdapter, 
                       elu_robot_arm_framework::RobotArmInterface)
```

### 4. 创建插件描述文件

```xml
<library path="your_robot_adapter">
  <class name="your_namespace/YourRobotAdapter" 
         type="your_namespace::YourRobotAdapter" 
         base_class_type="elu_robot_arm_framework::RobotArmInterface">
    <description>Your Robot Adapter Plugin</description>
  </class>
</library>
```

## 测试指南

### 单元测试

使用Google Test框架：

```cpp
TEST_F(YourAdapterTest, ConnectionTest) 
{
  EXPECT_TRUE(adapter_->connect("test_config.yaml"));
  EXPECT_TRUE(adapter_->isConnected());
}
```

### 集成测试

测试整个控制流程：

```bash
ros2 test src/elu_robot_arm_framework
```

## 调试技巧

### 启用详细日志

```bash
ros2 launch elu_robot_arm_framework elu_robot_framework.launch.py log_level:=debug
```

### 使用RQT工具

```bash
# 查看话题
rqt_graph

# 监控消息
rqt_topic

# 调用服务
rqt_service_caller
```
