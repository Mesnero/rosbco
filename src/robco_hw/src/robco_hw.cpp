// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <netdb.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "robco_hw/robco_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robcomm/robcomm.hpp"

namespace robco_hw
{
hardware_interface::CallbackReturn RobcoHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Set default parameter values
  robot_ip_ = info_.hardware_parameters["robot_ip"];
  local_rx_port_ = std::stoi(info_.hardware_parameters["local_rx_port"]);
  remote_tx_port_ = std::stoi(info_.hardware_parameters["remote_tx_port"]);
  robot_init_timeout_ = std::stoi(info_.hardware_parameters["robot_init_timeout"]);

  // Setup joints
  for (auto joint : info_.joints)
  {
    Joint j;
    j.name = joint.name;
    // Check if command interfaces are correctly configured
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"), "Joint '%s' has %ld command interfaces. 1 expected.",
                  joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"), "Joint '%s' has command interface '%s'. '%s' expected.",
                  joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"), "Joint '%s' has %ld state interfaces. 2 expected.",
                  joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"), "Joint '%s' has state interface '%s'. '%s' expected.",
                  joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"), "Joint '%s' has state interface '%s'. '%s' expected.",
                  joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.pos = std::stod(joint.state_interfaces[0].initial_value);
    j.vel = 0.0;
    j.vel_cmd = 0.0;
    joints_.push_back(j);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobcoHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"), "Attempting to connect to robot at %s (rx %d tx %d)...",
              robot_ip_.c_str(), local_rx_port_, remote_tx_port_);

  robot_.connect(robot_ip_, local_rx_port_, remote_tx_port_);

  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"),
              "Waiting for robot to initialize (timeout is %d seconds)...", robot_init_timeout_);

  for (int i = 0; i < 100 * robot_init_timeout_ && !robot_.is_initialized(); i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  if (!robot_.is_initialized())
  {
    RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"), "Timeout waiting for robot to initialize");
    return CallbackReturn::ERROR;
  }

  robot_.receive();

  if (robot_.get_joint_count() != (int) joints_.size())
  {
    RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"), "Robot has %d joints, but %ld were configured",
                 robot_.get_joint_count(), joints_.size());
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"), "%d joints found", robot_.get_joint_count());
  }
  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"), "Robco robot interface initialized.");
  robot_.set_state(robcomm::ROBOT_STATE_CMD_SWITCHED_ON);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobcoHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  robot_.set_state(robcomm::ROBOT_STATE_CMD_OPERATIONAL);

  if (!robot_.is_initialized())
  {
    RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"), "Robot not initialized!");
    return CallbackReturn::ERROR;
  }

  auto joint_angles = robot_.getJointAngles();
  for (size_t i = 0; i < joints_.size(); i++)
  {
    joints_[i].pos = joint_angles[i];
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobcoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  robot_.set_state(robcomm::ROBOT_STATE_CMD_DISABLED);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobcoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& joint : joints_)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint.pos));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.vel));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobcoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& joint : joints_)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.vel_cmd));
  }
  return command_interfaces;
}

hardware_interface::return_type RobcoHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    if (!robot_.is_initialized())
    {
        return hardware_interface::return_type::ERROR;
    }
    robot_.receive();
    auto joint_angles = robot_.getJointAngles();
    for (size_t i = 0; i < joints_.size(); i++)
    {
        joints_[i].vel = (joint_angles[i] - joints_[i].pos) / period.seconds();
        joints_[i].pos = joint_angles[i];
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobcoHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!robot_.is_initialized())
    {
        return hardware_interface::return_type::ERROR;
    }
    std::vector<double> dqs;
    for (auto& joint : joints_)
    {
        // robcomm expects rad/s but exectues command over 10 ms
        auto dq = joint.vel_cmd;
        dqs.push_back(dq);
    }
    robot_.jog_joints(dqs);
    return hardware_interface::return_type::OK;
}

}  // namespace robco_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robco_hw::RobcoHardwareInterface, hardware_interface::SystemInterface)
