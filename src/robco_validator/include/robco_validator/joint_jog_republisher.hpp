#ifndef JOINT_JOG_REPUBLISHER_HPP
#define JOINT_JOG_REPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace robco_validator
{

class JointJogRepublisher : public rclcpp::Node
{
public:

JointJogRepublisher(std::string topic = "servo/delta_joint_cmds")
  : Node("joint_jog_republisher")
  {
    api_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/vel_cmds",
        10,
        std::bind(&JointJogRepublisher::statusCallback, this, std::placeholders::_1));

    servo_pub_ =
        this->create_publisher<control_msgs::msg::JointJog>(topic, 10);
  }

private:
  void statusCallback(const std_msgs::msg::Float64MultiArray msg)
  {
    auto republished_msg = std::make_unique<control_msgs::msg::JointJog>();
    republished_msg->header.stamp = this->now();
    republished_msg->joint_names = {"drive1_joint", "drive2_joint", "drive3_joint","drive4_joint","drive5_joint","drive6_joint"};
    republished_msg->velocities = msg.data;
    servo_pub_->publish(std::move(republished_msg));
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr api_cmd_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr servo_pub_;
};

}  // namespace robco_validator

#endif  // ROBCO_VALIDATOR_SERVO_STATUS_REPUBLISHER_HPP
