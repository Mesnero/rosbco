#ifndef ROBCO_HW__ROBCO_HW_HPP_
#define ROBCO_HW__ROBCO_HW_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robco_hw/visibility_control.h"

#include "robcomm/robcomm.hpp"

namespace robco_hw {
class RobcoHardwareInterface : public hardware_interface::SystemInterface {
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Default configuration constants
  static constexpr const char *DEFAULT_ROBOT_IP = "192.168.0.1";
  static constexpr int DEFAULT_LOCAL_RX_PORT = 5000;
  static constexpr int DEFAULT_REMOTE_TX_PORT = 6000;
  static constexpr int DEFAULT_ROBOT_INIT_TIMEOUT = 10;

  // Internal joint representation
  struct Joint {
    std::string name;
    double pos{0.0};
    double vel{0.0};
    double vel_cmd{0.0};
  };

  robcomm::Robot robot_;
  std::vector<Joint> joints_;

  // Configuration parameters
  std::string robot_ip_;
  int local_rx_port_;
  int remote_tx_port_;
  int robot_init_timeout_;
};

} // namespace robco_hw

#endif // ROBCO_HW__ROBCO_HW_HPP_