/**
 * @file publisher_member_function.cpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @brief talker
 * @version 0.1
 * @date 2023-10-24
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <publisher.hpp>

/**
 * @brief Construct a new Minimal Publisher:: Minimal Publisher object
 *
 * @param node_name
 * @param topic_name
 */
MinimalPublisher::MinimalPublisher(const std::string& node_name,
                                   std::string topic_name)
    : Node(node_name) {
  // Declaring parameters
  this->declare_parameter("message", "Blink-182 rocks!");
  this->declare_parameter("message_freq", 1000);

  // Reading from parameters
  message_.data = this->get_parameter("message").as_string();
  int pub_freq = this->get_parameter("message_freq").as_int();

  // For Logging Printouts
  if (pub_freq < 500) {
    RCLCPP_FATAL(this->get_logger(), "Too high of frequency, aborting...");
    exit(2);
  } else if (pub_freq < 700) {
    RCLCPP_ERROR(this->get_logger(), "Better publish frequency!");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Starting the publisher...");
  }

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(pub_freq),
      std::bind(&MinimalPublisher::timer_callback, this));

  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

  service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
      "change_string", std::bind(&MinimalPublisher::change_string, this,
                                 std::placeholders::_1, std::placeholders::_2));
  
  MinimalPublisher::tf_static_broadcaster_  =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  this->make_transforms();
}

/**
 * @brief timer_callback
 *
 */
void MinimalPublisher::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
  publisher_->publish(message_);
}

/**
 * @brief change_string service
 *
 * @param request
 * @param response
 */
void MinimalPublisher::change_string(
    const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
        request,
    std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> response) {
  message_.data = request->after;
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Incoming request\nnew_string: %s",
              request->after.c_str());
  response->status = "STRING CHANGED!";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]",
              response->status.c_str());
}

void MinimalPublisher::make_transforms() {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "talk";

  t.transform.translation.x = 1;
  t.transform.translation.y = 1;
  t.transform.translation.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 10);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_static_broadcaster_->sendTransform(t);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<MinimalPublisher>("minimal_publisher", "topic"));
  rclcpp::shutdown();
  return 0;
}
