/**
 * @file integration_test.cpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @brief Level 2 Integration Test using Google Test
 * @version 0.1
 * @date 2023-10-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <std_msgs/msg/string.hpp>
// #include "std_msgs/msg/string.hpp"
#include <iostream>
// #include <gtest/gtest.h>

#include <chrono>
#include <stdlib.h>
#include <memory>
// #include <rclcpp/rclcpp.hpp>

// #include <publisher.hpp>
// #include "beginner_tutorials/srv/change_string.hpp"

using namespace std::chrono_literals;

// #include <publisher.hpp>
// #include "include/publisher.hpp"
// #include <include/publisher.hpp>
// #include "beginner_tutorials/include/publisher.hpp"
// #include <beginner_tutorials/srv/change_string.hpp>
// #include "beginner_tutorials/include/publisher.hpp"
// #include "beginner_tutorials/srv/ChangeString.srv"

/**
 * @brief TestingFixture Class for Level 2 Integration Test
 * 
 */
namespace beginner_tutorials {
class TestingFixture : public testing::Test, public rclcpp::Node {
public:
  /**
   * @brief Construct a new Testing Fixture object
   * 
   */
  TestingFixture() 
    : Node("integration_test") {
  // TestingFixture() : node_(std::make_shared<rclcpp::Node>("integration_test")) {
  
  RCLCPP_ERROR_STREAM(node_->get_logger(), "Testing Constructor Created...");
  }

  /**
   * @brief Set the SetUp object
   * 
   */
  void SetUp() override {
    rclcpp::init(0, nullptr);
    RCLCPP_INFO_STREAM(node_->get_logger(), "gTest SetUp Complete!");
  }

  /**
   * @brief TearDown method
   * 
   */
  void TearDown() override { std::cout << "TearDown conducted..." << std::endl; }

 private:
  rclcpp::Node::SharedPtr node_;

};

/**
 * @brief Construct a new test fixture object for ensuring fixture works
 * 
 */
TEST_F(TestingFixture, TrueIsTrueTest) {
  std::cout << "Commenceing Fixture Test" << std::endl;
  EXPECT_TRUE(true);
}

/**
 * @brief Construct a new test fixture object for Testing Publisher
 * 
 */
TEST_F(TestingFixture, test_pubcount) {
  // node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub = this->create_publisher<std_msgs::msg::String>
                    ("chatter", 10.0);

  auto num_pub = this->count_publishers("chatter");

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        "Output String:" << static_cast<int>(num_pub));

  ASSERT_EQ(1, static_cast<int>(num_pub));

}
} // namespace beginner_tutorials

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}