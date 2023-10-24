/**
 * @brief Modified by Jerry Pittman
 * 
 */

// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
// #include <chrono_literals>
#include <functional>

#include <publisher.hpp>

// #include "beginner_tutorials"
// #include "beginner_tutorials/srv/change_string.hpp"
// #include "beginner_tutorials/srv/change_string.h"
// #include "ChangString.srv"
// #include "srv/ChangString.srv"

// #include "msg/string.hpp"
using namespace std::chrono_literals;

MinimalPublisher::MinimalPublisher(const std::string &node_name,
                                   std::string topic_name)
    : Node(node_name) {
  // Declaring parameters
  // this->declare_parameter("my_message", "blink-182!");
  // Receiving Parameters
  // message_.data = this->get_parameter("my_message").as_string();

  timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

  service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
    "change_string", std::bind(&MinimalPublisher::change_string, this,
                                std::placeholders::_1, std::placeholders::_2));
  auto message = std_msgs::msg::String();
  message_.data = "Blink-182 rocks! ";
}

void MinimalPublisher::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
  publisher_->publish(message_);
}

void MinimalPublisher::change_string(const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request> request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> response) {
  message_.data = request->after;
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Incoming request\nnew_string: %s",
              request->after.c_str());
  response->status = "STRING CHANGED!";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]",
              response->status.c_str());
}


int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>("minimal_publisher", "topic"));
  rclcpp::shutdown();
  return 0;
}
