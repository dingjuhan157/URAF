/**
 * @file test_plugin_manager.cpp
 * @brief 插件管理器单元测试
 */

#include <gtest/gtest.h>
#include "elu_robot_arm_framework/utils/plugin_manager.hpp"

class PluginManagerTest : public ::testing::Test 
{
protected:
  void SetUp() override {
    plugin_manager_ = std::make_unique<elu_robot_arm_framework::PluginManager>();
  }

  void TearDown() override {
    plugin_manager_.reset();
  }

  std::unique_ptr<elu_robot_arm_framework::PluginManager> plugin_manager_;
};

TEST_F(PluginManagerTest, InitializationTest) 
{
  EXPECT_NE(plugin_manager_, nullptr);
}

TEST_F(PluginManagerTest, LoadPluginTest) 
{
  // 测试加载不存在的插件
  bool result = plugin_manager_->loadPlugin("test_robot", "non_existent_plugin");
  EXPECT_FALSE(result);
}

TEST_F(PluginManagerTest, GetAvailablePluginsTest) 
{
  auto available_plugins = plugin_manager_->getAvailablePlugins();
  // 至少应该有ELU适配器插件
  EXPECT_TRUE(available_plugins.size() >0);
}
