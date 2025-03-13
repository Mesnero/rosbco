#ifndef JOINT_JOG_REPUBLISHER_HPP
#define JOINT_JOG_REPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <vector>

namespace robco_validator
{

class JointJogRepublisher : public rclcpp::Node
{
public:

  JointJogRepublisher(const std::string & topic = "servo/delta_joint_cmds")
  : Node("joint_jog_republisher"),
    collision_scale_(1.0)
  {
    collision_scale_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/validator_node/collision_velocity_scale", 10,
      std::bind(&JointJogRepublisher::collisionScaleCallback, this, std::placeholders::_1));

    api_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/vel_cmds", 10,
      std::bind(&JointJogRepublisher::commandCallback, this, std::placeholders::_1));

    servo_pub_ = this->create_publisher<control_msgs::msg::JointJog>(topic, 10);
  }

private:
  void collisionScaleCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    collision_scale_ = msg->data;
  }

  void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    auto republished_msg = std::make_unique<control_msgs::msg::JointJog>();
    republished_msg->header.stamp = this->now();
    republished_msg->joint_names = {"drive1_joint", "drive2_joint", "drive3_joint",
                                    "drive4_joint", "drive5_joint", "drive6_joint"};

    if (collision_scale_ < 0.01)
    {
      republished_msg->velocities.assign(msg->data.size(), 0.0);
    }
    else
    {
      // Otherwise, pass the incoming command as is.
      republished_msg->velocities = msg->data;
    }

    servo_pub_->publish(std::move(republished_msg));
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr api_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr collision_scale_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr servo_pub_;
  double collision_scale_; 
};

}  // namespace robco_validator

#endif  // JOINT_JOG_REPUBLISHER_HPP
