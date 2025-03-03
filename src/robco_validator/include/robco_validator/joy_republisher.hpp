#ifndef JOY_REPUBLISHER_HPP
#define JOY_REPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};

enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

namespace robco_validator
{

class JoyRepublisher : public rclcpp::Node
{
public:

JoyRepublisher(std::string topic = "servo/delta_twist_cmds", std::string frame_id = "world")
  : Node("joy_republisher"), frame_id_{frame_id}
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/gamepad_cmds",
        10,
        std::bind(&JoyRepublisher::statusCallback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic, 10);
  }

private:
  void statusCallback(const sensor_msgs::msg::Joy msg)
  {
    auto republished_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    republished_msg->header.stamp = this->now();
    republished_msg->header.frame_id = frame_id_;
    republished_msg->twist.linear.z = msg.axes[RIGHT_STICK_Y];
    republished_msg->twist.linear.y = msg.axes[RIGHT_STICK_X];
    republished_msg->twist.linear.x = msg.axes[RIGHT_TRIGGER] - msg.axes[LEFT_TRIGGER];

    republished_msg->twist.angular.y = msg.axes[LEFT_STICK_Y];
    republished_msg->twist.angular.x = msg.axes[LEFT_STICK_X];
    republished_msg->twist.angular.z = msg.buttons[RIGHT_BUMPER] - msg.buttons[LEFT_BUMPER];

    twist_pub_->publish(std::move(republished_msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::string frame_id_;
};

}  // namespace robco_validator

#endif  // JOY_REPUBLISHER_HPP
