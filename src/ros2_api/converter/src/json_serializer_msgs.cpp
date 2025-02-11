#include <ros2_api/converter/json_serializer_msgs.hpp>
#include <ros2_api_msgs/msg/calculated_states.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/time.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
namespace ros2_api {
    namespace converter {

    void to_json(json& j, const JointTrajectory w) {
        j["joint_traj_points"] = json::array();
        for (auto &trajPoint: w.trajectoryMsg.points) {
            json point = 
                json {
                    {"positions", trajPoint.positions},
                    {"velocities", trajPoint.velocities},
                    {"accelerations", trajPoint.accelerations},
                    {"effort", trajPoint.effort},
                    {"seconds", trajPoint.time_from_start.sec},
                    {"nanoseconds", (int) trajPoint.time_from_start.sec}
                };
            j["joint_traj_points"].push_back(point);
        }
    }

    void from_json(const json& j, JointTrajectory& w) {
        auto joint_names = config::ConfigParser::instance().getJointNames();
        auto frame_id = config::ConfigParser::instance().getBaseFrame();
        trajectory_msgs::msg::JointTrajectory trajMsg;
        trajMsg.header.stamp = rclcpp::Time(0,0, RCL_ROS_TIME); 
        trajMsg.header.frame_id = frame_id;
        trajMsg.joint_names = joint_names;
        std::vector<json> points;
        j.at("joint_traj_points").get_to(points);
        for (json trajPoint : points) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            trajPoint.at("positions").get_to(point.positions);
            trajPoint.at("velocities").get_to(point.velocities);
            trajPoint.at("accelerations").get_to(point.accelerations);
            trajPoint.at("effort").get_to(point.effort);
            builtin_interfaces::msg::Duration duration;
            trajPoint.at("nanoseconds").get_to(duration.nanosec);
            trajPoint.at("seconds").get_to(duration.sec);
            point.time_from_start = duration;
            trajMsg.points.push_back(point);
        }
        w.trajectoryMsg = trajMsg;
    }

    void to_json(json& j, const JointGroupController g) {
        j["joint_values"] = g.positions.data;
    }

    void from_json(const json& j, JointGroupController& g) {
        j.at("joint_values").get_to(g.positions.data);
        g.positions.layout.dim.resize(1);               
        g.positions.layout.dim[0].label = "data";   
        g.positions.layout.dim[0].size = g.positions.data.size();
        g.positions.layout.dim[0].stride = g.positions.data.size();
        g.positions.layout.data_offset = 0; 
    }

    void to_json(json& j, const CalculatedStates c) {
        j["names"] = c.states.name;
        j["position_angle"] = c.states.position_angle;
        j["acceleration"] = c.states.acceleration;
        j["velocity"] = c.states.velocity;
        j["jerk"] = c.states.jerk;
        j["position_absolute"] = json::array();
        std::vector<geometry_msgs::msg::Point> points = c.states.position_space;
        for(auto point : points) {
            j["position_absolute"].push_back(json{
                {"x", point.x},
                {"y", point.y},
                {"z", point.z}
            });
        }
    }

    void from_json(const json& j, CalculatedStates& c) {
        j.at("names").get_to(c.states.name);
        j.at("position_angle").get_to(c.states.position_angle);
        j.at("velocity").get_to(c.states.velocity);
        j.at("acceleration").get_to(c.states.acceleration);
        j.at("jerk").get_to(c.states.jerk);
        std::vector<json> points;
        j.at("position_space").get_to(points);
        for(json point: points) {
            geometry_msgs::msg::Point p;
            point.at("x").get_to(p.x);
            point.at("y").get_to(p.y);
            point.at("z").get_to(p.z);
            c.states.position_space.push_back(p);
        }
    }

    void to_json(json& j, const Feedback f) {
        j["feedback_code"] = f.feedback.status_code;
        j["message"] = f.feedback.message;
    }

    void from_json(const json& j, Feedback& f) {
        j.at("feedback_code").get_to(f.feedback.status_code);
        j.at("message").get_to(f.feedback.message);
    }

    void to_json(json& j, const JointStates js) {
        j["names"] = js.states.name;
        j["position"] = js.states.position;
        j["velocity"] = js.states.velocity;
        j["effort"] = js.states.effort;
    }

    void from_json(const json& j, JointStates& js) {
        auto joint_names = config::ConfigParser::instance().getJointNames();
        auto frame_id = config::ConfigParser::instance().getBaseFrame();
        js.states.header.stamp = rclcpp::Time(0,0, RCL_ROS_TIME); 
        js.states.header.frame_id = frame_id;
        js.states.name = joint_names;
        j.at("position").get_to(js.states.position);
        j.at("effort").get_to(js.states.effort);
        j.at("velocity").get_to(js.states.velocity);
    }
    } // namespace converter
} // namespace ros2_api