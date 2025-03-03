#ifndef SERVO_STATUS_REPUBLISHER_HPP
#define SERVO_STATUS_REPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>

#include <moveit_servo/status_codes.h>     

namespace robco_validator
{

class ServoStatusRepublisher : public rclcpp::Node
{
public:

  ServoStatusRepublisher(std::string topic = "moveit_servo/status")
  : Node("servo_status_republisher_node")
  {
    status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        topic,
        10,
        std::bind(&ServoStatusRepublisher::statusCallback, this, std::placeholders::_1));

    feedback_pub_ =
        this->create_publisher<ros2_api_msgs::msg::ClientFeedback>("client_feedback", 10);
  }

private:
  void statusCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    ros2_api_msgs::msg::ClientFeedback republished_msg;

    auto it = moveit_servo::SERVO_STATUS_CODE_MAP.find(
        static_cast<moveit_servo::StatusCode>(msg->data));
    if (it != moveit_servo::SERVO_STATUS_CODE_MAP.end())
    {
      republished_msg.message = it->second;
    }
    else
    {
      republished_msg.message = "Unknown status code";
    }

    // Add 101 to the code to make it fit to the enum in ros2_api/types.hpp
    republished_msg.status_code = msg->data + 101;

    // Publish on "client_feedback"
    feedback_pub_->publish(republished_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr status_sub_;
  rclcpp::Publisher<ros2_api_msgs::msg::ClientFeedback>::SharedPtr feedback_pub_;
};

}  // namespace robco_validator

#endif  // SERVO_STATUS_REPUBLISHER_HPP
