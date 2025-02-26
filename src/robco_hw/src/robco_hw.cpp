#include <chrono>
#include <cmath>
#include <netdb.h>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robco_hw/robco_hw.hpp"
#include "robcomm/robcomm.hpp"

namespace robco_hw {
hardware_interface::CallbackReturn
RobcoHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
  // Call base class initialization.
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Use default values if parameters are not provided.
  if (info_.hardware_parameters.count("robot_ip") > 0) {
    robot_ip_ = info_.hardware_parameters.at("robot_ip");
  } else {
    robot_ip_ = DEFAULT_ROBOT_IP;
  }

  if (info_.hardware_parameters.count("local_rx_port") > 0) {
    local_rx_port_ = std::stoi(info_.hardware_parameters.at("local_rx_port"));
  } else {
    local_rx_port_ = DEFAULT_LOCAL_RX_PORT;
  }

  if (info_.hardware_parameters.count("remote_tx_port") > 0) {
    remote_tx_port_ = std::stoi(info_.hardware_parameters.at("remote_tx_port"));
  } else {
    remote_tx_port_ = DEFAULT_REMOTE_TX_PORT;
  }

  if (info_.hardware_parameters.count("robot_init_timeout") > 0) {
    robot_init_timeout_ =
        std::stoi(info_.hardware_parameters.at("robot_init_timeout"));
  } else {
    robot_init_timeout_ = DEFAULT_ROBOT_INIT_TIMEOUT;
  }

  // Setup joints based on the provided hardware info.
  for (const auto &joint_info : info_.joints) {
    Joint j;
    j.name = joint_info.name;

    // Check command interfaces: expect exactly one and it must be velocity.
    if (joint_info.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"),
                   "Joint '%s' has %ld command interfaces. 1 expected.",
                   joint_info.name.c_str(),
                   joint_info.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint_info.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"),
                   "Joint '%s' has command interface '%s'. '%s' expected.",
                   joint_info.name.c_str(),
                   joint_info.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check state interfaces: expect exactly two, position then velocity.
    if (joint_info.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"),
                   "Joint '%s' has %ld state interfaces. 2 expected.",
                   joint_info.name.c_str(), joint_info.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint_info.state_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"),
                   "Joint '%s' has state interface '%s'. '%s' expected.",
                   joint_info.name.c_str(),
                   joint_info.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint_info.state_interfaces[1].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"),
                   "Joint '%s' has state interface '%s'. '%s' expected.",
                   joint_info.name.c_str(),
                   joint_info.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize joint state using the provided initial value for position.
    j.pos = std::stod(joint_info.state_interfaces[0].initial_value);
    j.vel = 0.0;
    j.vel_cmd = 0.0;
    joints_.push_back(j);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobcoHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"),
              "Attempting to connect to robot at %s (rx %d tx %d)...",
              robot_ip_.c_str(), local_rx_port_, remote_tx_port_);
  robot_.connect(robot_ip_, local_rx_port_, remote_tx_port_);

  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"),
              "Waiting for robot to initialize (timeout is %d seconds)...",
              robot_init_timeout_);

  for (int i = 0; i < 100 * robot_init_timeout_ && !robot_.is_initialized(); i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  if (!robot_.is_initialized()) {
    RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"),
                 "Timeout waiting for robot to initialize");
    return CallbackReturn::ERROR;
  }

  robot_.receive();

  if (robot_.get_joint_count() != (int)joints_.size()) {
    RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"),
                 "Robot has %d joints, but %ld were configured",
                 robot_.get_joint_count(), joints_.size());
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"), "%d joints found",
                robot_.get_joint_count());
  }
  RCLCPP_INFO(rclcpp::get_logger("RobcoHardwareInterface"),
              "Robco robot interface initialized.");
  robot_.set_state(robcomm::ROBOT_STATE_CMD_SWITCHED_ON);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobcoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  robot_.set_state(robcomm::ROBOT_STATE_CMD_OPERATIONAL);

  if (!robot_.is_initialized()) {
    RCLCPP_FATAL(rclcpp::get_logger("RobcoHardwareInterface"),
                 "Robot not initialized!");
    return CallbackReturn::ERROR;
  }

  auto joint_angles = robot_.getJointAngles();
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].pos = joint_angles[i];
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobcoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  robot_.set_state(robcomm::ROBOT_STATE_CMD_DISABLED);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RobcoHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto &joint : joints_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.vel));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobcoHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto &joint : joints_) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.vel_cmd));
  }
  return command_interfaces;
}

hardware_interface::return_type
RobcoHardwareInterface::read(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration &period) {
  if (!robot_.is_initialized()) {
    return hardware_interface::return_type::ERROR;
  }
  robot_.receive();
  auto joint_angles = robot_.getJointAngles();
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].vel = (joint_angles[i] - joints_[i].pos) / period.seconds();
    joints_[i].pos = joint_angles[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
RobcoHardwareInterface::write(const rclcpp::Time & /*time*/,
                              const rclcpp::Duration & /*period*/) {
  if (!robot_.is_initialized()) {
    return hardware_interface::return_type::ERROR;
  }
  std::vector<double> dqs;
  for (auto &joint : joints_) {
    // robcomm exectues command over 10 ms
    auto dq = joint.vel_cmd;
    dqs.push_back(dq);
  }
  robot_.jog_joints(dqs);
  return hardware_interface::return_type::OK;
}

} // namespace robco_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robco_hw::RobcoHardwareInterface,
                       hardware_interface::SystemInterface)
