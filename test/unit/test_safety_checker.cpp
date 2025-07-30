/**
 * @file test_safety_checker.cpp
 * @brief 安全检查器单元测试
 */

#include <gtest/gtest.h>
#include "elu_robot_arm_framework/safety/safety_checker.hpp"

class SafetyCheckerTest : public ::testing::Test 
{
protected:
  void SetUp() override {
    safety_checker_ = std::make_unique<elu_robot_arm_framework::SafetyChecker>();
  }

  void TearDown() override {
    safety_checker_.reset();
  }

  std::unique_ptr<elu_robot_arm_framework::SafetyChecker> safety_checker_;
};

TEST_F(SafetyCheckerTest, InitializationTest) 
{
  EXPECT_NE(safety_checker_, nullptr);
  EXPECT_TRUE(safety_checker_->isSafetyCheckEnabled());
}

TEST_F(SafetyCheckerTest, SpeedCheckTest) 
{
  // 测试正常速度
  auto result = safety_checker_->checkSpeed(0.5);
  EXPECT_TRUE(result.is_safe);

  // 测试超出范围的速度
  result = safety_checker_->checkSpeed(1.5);
  EXPECT_FALSE(result.is_safe);

  result = safety_checker_->checkSpeed(-0.1);
  EXPECT_FALSE(result.is_safe);
}

TEST_F(SafetyCheckerTest, WorkspaceCheckTest) 
{
  geometry_msgs::msg::Pose pose;
  
  // 测试工作空间内的位置
  pose.position.x = 0.5;
  pose.position.y = 0.5;
  pose.position.z = 0.5;
  
  auto result = safety_checker_->checkWorkspace(pose);
  EXPECT_TRUE(result.is_safe);

  // 测试工作空间外的位置
  pose.position.x = 10.0;
  result = safety_checker_->checkWorkspace(pose);
  EXPECT_FALSE(result.is_safe);
}
