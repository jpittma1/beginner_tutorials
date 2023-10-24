#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "beginner_tutorials/srv/change_string.hpp"

class MinimalPublisher : public rclcpp::Node {
 public:
    MinimalPublisher(const std::string &node_name, std::string topic_name);


 private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std_msgs::msg::String message_;  
    rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;  
  
    void timer_callback();

    void change_string(
        const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request> request,
        std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> response);
};