/**
 * @file publisher.hpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @brief header file for talker
 * @version 0.1
 * @date 2023-10-24
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
// #include <chrono_literals>

#include "beginner_tutorials/srv/change_string.hpp"

using namespace std::chrono_literals;

/**
 * @brief  MinimalPublisher Class
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   * @param node_name
   * @param topic_name
   */
  MinimalPublisher(const std::string &node_name, std::string topic_name);

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std_msgs::msg::String message_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;

  void timer_callback();

  void change_string(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response);
};