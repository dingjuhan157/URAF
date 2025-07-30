/**
 * @file plugin_manager.hpp
 * @brief 插件管理器
 * @author ELU Robotics Team
 * @date 2025
 */

#ifndef ELU_ROBOT_ARM_FRAMEWORK__UTILS__PLUGIN_MANAGER_HPP_
#define ELU_ROBOT_ARM_FRAMEWORK__UTILS__PLUGIN_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include "elu_robot_arm_framework/interfaces/robot_arm_interface.hpp"

namespace elu_robot_arm_framework
{

/**
 * @brief 插件管理器
 * 负责动态加载和管理机械臂适配器插件
 */
class PluginManager
{
public:
  PluginManager();
  virtual ~PluginManager();

  // 插件管理
  bool loadPlugin(const std::string& robot_id, const std::string& plugin_name);
  std::shared_ptr<RobotArmInterface> getPlugin(const std::string& robot_id);
  bool unloadPlugin(const std::string& robot_id);
  std::vector<std::string> getLoadedPlugins() const;
  std::vector<std::string> getAvailablePlugins() const;

  // 信息查询
  bool isPluginLoaded(const std::string& robot_id) const;
  std::string getPluginType(const std::string& robot_id) const;

private:
  std::unique_ptr<pluginlib::ClassLoader<RobotArmInterface>> plugin_loader_;
  std::unordered_map<std::string, std::shared_ptr<RobotArmInterface>> loaded_plugins_;
  std::unordered_map<std::string, std::string> plugin_types_;
  
  rclcpp::Logger logger_;
};

} // namespace elu_robot_arm_framework

#endif // ELU_ROBOT_ARM_FRAMEWORK__UTILS__PLUGIN_MANAGER_HPP_
