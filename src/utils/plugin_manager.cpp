/**
 * @file plugin_manager.cpp
 * @brief 插件管理器实现
 * @author ELU Robotics Team
 * @date 2025
 */

#include "elu_robot_arm_framework/utils/plugin_manager.hpp"

namespace elu_robot_arm_framework
{

PluginManager::PluginManager()
: logger_(rclcpp::get_logger("plugin_manager"))
{
  try {
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<RobotArmInterface>>(
      "elu_robot_arm_framework", "elu_robot_arm_framework::RobotArmInterface");
    
    RCLCPP_INFO(logger_, "Plugin manager initialized successfully");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to initialize plugin loader: %s", e.what());
  }
}

PluginManager::~PluginManager()
{
  // 卸载所有插件
  for (auto& [robot_id, plugin] : loaded_plugins_) {
    if (plugin && plugin->isConnected()) {
      plugin->disconnect();
    }
  }
  loaded_plugins_.clear();
}

bool PluginManager::loadPlugin(const std::string& robot_id, const std::string& plugin_name)
{
  try {
    // 检查是否已经加载
    if (isPluginLoaded(robot_id)) {
      RCLCPP_WARN(logger_, "Plugin already loaded for robot: %s", robot_id.c_str());
      return true;
    }

    // 创建插件实例
    auto plugin = plugin_loader_->createSharedInstance(plugin_name);
    if (!plugin) {
      RCLCPP_ERROR(logger_, "Failed to create plugin instance: %s", plugin_name.c_str());
      return false;
    }

    // 保存插件
    loaded_plugins_[robot_id] = plugin;
    plugin_types_[robot_id] = plugin_name;

    RCLCPP_INFO(logger_, "Plugin loaded successfully: %s -> %s", 
                robot_id.c_str(), plugin_name.c_str());
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception while loading plugin %s: %s", 
                 plugin_name.c_str(), e.what());
    return false;
  }
}

std::shared_ptr<RobotArmInterface> PluginManager::getPlugin(const std::string& robot_id)
{
  auto it = loaded_plugins_.find(robot_id);
  if (it != loaded_plugins_.end()) {
    return it->second;
  }
  return nullptr;
}

bool PluginManager::unloadPlugin(const std::string& robot_id)
{
  auto it = loaded_plugins_.find(robot_id);
  if (it == loaded_plugins_.end()) {
    RCLCPP_WARN(logger_, "Plugin not found for robot: %s", robot_id.c_str());
    return false;
  }

  // 断开连接
  if (it->second && it->second->isConnected()) {
    it->second->disconnect();
  }

  // 移除插件
  loaded_plugins_.erase(it);
  plugin_types_.erase(robot_id);

  RCLCPP_INFO(logger_, "Plugin unloaded successfully: %s", robot_id.c_str());
  return true;
}

std::vector<std::string> PluginManager::getLoadedPlugins() const
{
  std::vector<std::string> loaded;
  for (const auto& [robot_id, plugin] : loaded_plugins_) {
    loaded.push_back(robot_id);
  }
  return loaded;
}

std::vector<std::string> PluginManager::getAvailablePlugins() const
{
  if (!plugin_loader_) {
    return {};
  }

  return plugin_loader_->getDeclaredClasses();
}

bool PluginManager::isPluginLoaded(const std::string& robot_id) const
{
  return loaded_plugins_.find(robot_id) != loaded_plugins_.end();
}

std::string PluginManager::getPluginType(const std::string& robot_id) const
{
  auto it = plugin_types_.find(robot_id);
  if (it != plugin_types_.end()) {
    return it->second;
  }
  return "";
}

} // namespace elu_robot_arm_framework
