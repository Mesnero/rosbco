#ifndef JSON_CONVERTERS_HPP
#define JSON_CONVERTERS_HPP

#include <string>
#include <vector>
#include <ros2_api_msgs/msg/calculated_states.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/time.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace ros2_api {
    namespace converter {

    struct JointTrajectory {
        trajectory_msgs::msg::JointTrajectory trajectoryMsg;
    };

    void to_json(json& j, const JointTrajectory w);
    void from_json(const json& j, JointTrajectory& w);

    struct JointGroupController {
        std_msgs::msg::Float64MultiArray positions;
    };

    void to_json(json& j, const JointGroupController g);
    void from_json(const json& j, JointGroupController& g);

    struct CalculatedStates {
        ros2_api_msgs::msg::CalculatedStates states;
    };

    void to_json(json& j, const CalculatedStates c);
    void from_json(const json& j, CalculatedStates& c);

    struct JointStates {
        sensor_msgs::msg::JointState states;
    };
    
    void to_json(json& j, const JointStates js);
    void from_json(const json& j, JointStates& js);

    struct Feedback {
        ros2_api_msgs::msg::ClientFeedback feedback;
    };

    void to_json(json& j, const Feedback f);
    void from_json(const json& j, Feedback& f);

    } // namespace converter
} // namespace ros2_api

#endif // JSON_CONVERTERS_HPP
