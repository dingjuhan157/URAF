/**
 * @file motion_controller_node.cpp
 * @brief 运动控制器节点
 * @author ELU Robotics Team
 * @date 2025
 */

#include <rclcpp/rclcpp.hpp>
#include "elu_robot_arm_framework/controllers/motion_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions options;
    auto motion_controller = std::make_shared<elu_robot_arm_framework::MotionController>(options);

    RCLCPP_INFO(motion_controller->get_logger(), "Motion Controller Node started");

    // 使用多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(motion_controller);
    executor.spin();
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("motion_controller_node"), 
                 "Exception in motion controller node: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
