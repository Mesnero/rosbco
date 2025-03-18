#ifndef JOINT_JOG_REPUBLISHER_HPP
#define JOINT_JOG_REPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <ros2_api_msgs/msg/client_feedback.hpp>

namespace robco_validator
{

class JointJogRepublisher : public rclcpp::Node
{
public:
  // The constructor now takes a parameter_node which should have the YAML parameters loaded.
  JointJogRepublisher(
    const std::string & topic = "servo/delta_joint_cmds",
    std::shared_ptr<rclcpp::Node> parameter_node = nullptr)
  : Node("joint_jog_republisher"),
    collision_scale_(1.0)
  {
    joint_names_ = {"drive1_joint", "drive2_joint", "drive3_joint", 
                    "drive4_joint", "drive5_joint", "drive6_joint"};
    auto params = parameter_node->list_parameters({}, 10);

    for (const auto & joint : joint_names_)
    {
      std::string param_name = "robot_description_planning.joint_limits." + joint + ".max_velocity";
      double max_vel = 4.0;  // Default value if the parameter isn't set.
      if (parameter_node && parameter_node->has_parameter(param_name)) {
        parameter_node->get_parameter(param_name, max_vel);
      }
      velocity_limits_.push_back(max_vel - 0.05);
    }

    collision_scale_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/validator_node/collision_velocity_scale", 10,
      std::bind(&JointJogRepublisher::collisionScaleCallback, this, std::placeholders::_1));

    api_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/vel_cmds", 10,
      std::bind(&JointJogRepublisher::commandCallback, this, std::placeholders::_1));

    servo_pub_ = this->create_publisher<control_msgs::msg::JointJog>(topic, 10);
    feedback_pub_ = this->create_publisher<ros2_api_msgs::msg::ClientFeedback>("feedback_channel", 10);
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
    republished_msg->joint_names = joint_names_;

    if (collision_scale_ < 0.01)
    {
      republished_msg->velocities.assign(msg->data.size(), 0.0);
      ros2_api_msgs::msg::ClientFeedback feedback_msg;
      feedback_msg.status_code = 105;
      feedback_msg.message = "Halted due to upcoming collision";
      feedback_pub_->publish(feedback_msg);
    }
    else
    {
      for (size_t i = 0; i < msg->data.size() && i < velocity_limits_.size(); i++)
      {
        double sign = (msg->data[i] > 0.0) ? 1.0 : -1.0;
        if (std::abs(msg->data[i]) > velocity_limits_[i])
        {
          republished_msg->velocities.push_back(sign * velocity_limits_[i]);
          ros2_api_msgs::msg::ClientFeedback feedback_msg;
          feedback_msg.status_code = 101;
          feedback_msg.message = "Velocity limit exceeded, sent maximum velocity";
          feedback_pub_->publish(feedback_msg);
        }
        else
        {
          republished_msg->velocities.push_back(msg->data[i]);
        }
      }
    }

    servo_pub_->publish(std::move(republished_msg));
  }

  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr api_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr collision_scale_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr servo_pub_;
  rclcpp::Publisher<ros2_api_msgs::msg::ClientFeedback>::SharedPtr feedback_pub_;
  double collision_scale_;
};

}  // namespace robco_validator

#endif  // JOINT_JOG_REPUBLISHER_HPP
