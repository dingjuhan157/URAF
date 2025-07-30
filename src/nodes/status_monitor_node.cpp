/**
 * @file status_monitor_node.cpp
 * @brief 状态监控节点
 * @author ELU Robotics Team
 * @date 2025
 */

#include <rclcpp/rclcpp.hpp>
#include "elu_robot_arm_framework/msg/robot_status.hpp"
#include <std_msgs/msg/string.hpp>

class StatusMonitor : public rclcpp::Node
{
public:
  StatusMonitor() : Node("status_monitor")
  {
    // 订阅机械臂状态
    status_subscriber_ = this->create_subscription<elu_robot_arm_framework::msg::RobotStatus>(
      "robot_status", 10,
      std::bind(&StatusMonitor::statusCallback, this, std::placeholders::_1));

    // 发布系统状态
    system_status_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "system_status", 10);

    // 创建状态检查定时器
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&StatusMonitor::statusTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Status Monitor initialized");
  }

private:
  void statusCallback(const elu_robot_arm_framework::msg::RobotStatus::SharedPtr msg)
  {
    // 处理机械臂状态
    if (msg->status == 4) { // ERROR状态
      RCLCPP_WARN(get_logger(), "Robot %s in ERROR state: %s", 
                  msg->robot_id.c_str(), msg->error_message.c_str());
    }
    else if (msg->status == 5) { // EMERGENCY_STOP状态
      RCLCPP_ERROR(get_logger(), "Robot %s in EMERGENCY_STOP state", 
                   msg->robot_id.c_str());
    }

    latest_status_ = *msg;
  }

  void statusTimerCallback()
  {
    // 发布系统状态摘要
    std_msgs::msg::String system_msg;
    system_msg.data = "System operational";
    system_status_publisher_->publish(system_msg);
  }

  rclcpp::Subscription<elu_robot_arm_framework::msg::RobotStatus>::SharedPtr status_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  elu_robot_arm_framework::msg::RobotStatus latest_status_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto status_monitor = std::make_shared<StatusMonitor>();

  RCLCPP_INFO(status_monitor->get_logger(), "Status Monitor Node started");

  rclcpp::spin(status_monitor);
  rclcpp::shutdown();
  return 0;
}
