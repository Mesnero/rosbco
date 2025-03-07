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

JoyRepublisher(std::string topic = "/servo/twist_cmd", std::string frame_id = "world")
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

    auto right_stick_y = msg.axes[RIGHT_STICK_Y];
    republished_msg->twist.linear.z = adjust_for_deadzone(right_stick_y);
    
    auto right_stick_x = msg.axes[RIGHT_STICK_X];
    republished_msg->twist.linear.y = adjust_for_deadzone(right_stick_x);

    // Both have values from -1 to 1 -> normalize
    auto normalized_right_trigger = 0.5 * (msg.axes[RIGHT_TRIGGER] + 1); // 0 - 1 
    auto normalized_left_trigger = -0.5 * (msg.axes[LEFT_TRIGGER] + 1); // -1 - 0
    republished_msg->twist.linear.x = adjust_for_deadzone(normalized_right_trigger) + adjust_for_deadzone(normalized_left_trigger);

    auto left_stick_y = msg.axes[LEFT_STICK_Y];
    auto left_stick_x = msg.axes[LEFT_STICK_X];
    republished_msg->twist.angular.y = adjust_for_deadzone(left_stick_y);
    republished_msg->twist.angular.x = adjust_for_deadzone(left_stick_x);
    republished_msg->twist.angular.z = msg.buttons[RIGHT_BUMPER] - msg.buttons[LEFT_BUMPER];

    twist_pub_->publish(std::move(republished_msg));
  }

  double adjust_for_deadzone(double axisValue) {
    auto deadzone = 0.1;
    if (axisValue < deadzone) {
      return 0.0;
    }
    return axisValue;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::string frame_id_;
};

}  // namespace robco_validator

#endif  // JOY_REPUBLISHER_HPP
