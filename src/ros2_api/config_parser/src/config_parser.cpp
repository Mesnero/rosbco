#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ros2_api/types/types.hpp>
#include <ros2_api/config_parser/config_parser.hpp>

namespace ros2_api {
namespace config {    
   
    ConfigParser::ConfigParser()
        : logger_(rclcpp::get_logger("ConfigParser"))
    {}

    ConfigParser &ConfigParser::instance()
    {
        static ConfigParser instance;
        return instance;
    }

    bool ConfigParser::load(const std::string &config_path)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        try
        {
            root_node_ = YAML::LoadFile(config_path);
            config_path_ = config_path;
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(logger_, "Error loading YAML file: %s", e.what());
            reset();    

            return false;
        }
        return validate_config();
    }

    void ConfigParser::reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        config_path_ = "";
        root_node_ = YAML::Node();
        publisherConfig_.clear();
        transport_plugin_name_ = "";
        transport_params_ = YAML::Node();
        state_topic_ = "";
    }

    std::string ConfigParser::getTransportType() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return transport_plugin_name_;
    }

    std::vector<MessageConfig> ConfigParser::getPublisherConfig() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return publisherConfig_;
    }

    std::string ConfigParser::ConfigParser::getStateTopic() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_topic_;
    }

    YAML::Node ConfigParser::getTransportParams() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return transport_params_;
    }

    std::vector<std::string> ConfigParser::getJointNames() const 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return joint_names_;
    }

    std::string ConfigParser::getBaseFrame() const 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return base_frame_;
    }

    bool ConfigParser::useCalculatedStates() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return use_calculated_states_;
    }

    bool ConfigParser::validate_config()
    {
        // Check if the root node has the necessary keys
        if (!root_node_["ros2_api"] || !root_node_["ros2_api"]["ros__parameters"])
        {
            RCLCPP_ERROR(logger_, "Missing 'ros2_api' or 'ros__parameters' in YAML.");
            return false;
        }
        YAML::Node params = root_node_["ros2_api"]["ros__parameters"];

        // Check if the state topic exists, if not use default
        state_topic_ = getTopicValue(params, "states_topic", "joint_states");

        if (params["use_calculated_states"]) {
            use_calculated_states_ = params["use_calculated_states"].as<bool>();
        } else {
            use_calculated_states_ = false;
        }


        if (params["joint_names"].IsSequence()) {
            for (const auto &iface: params["joint_names"]) {
                joint_names_.push_back(iface.as<std::string>());
            }
        } else {
            RCLCPP_WARN(logger_, "No joint_names specified! JointTrajectory will not work.");
        }

        if (params["base_frame"]) {
            base_frame_ = params["base_frame"].as<std::string>();
        } 
        else {
            base_frame_ = "map";
        }

        // Check if command interfaces are defined and an array
        if (!params["publishers"] || !params["publishers"].IsSequence())
        {
            RCLCPP_ERROR(logger_, "Invalid or missing publishers in YAML.");
            return false;
        }

        for (const auto &iface : params["publishers"])
        {
            MessageConfig cfg;
            if (!iface["publisher"]) {
                RCLCPP_ERROR(logger_, "No publisher specified. Skipping.");
                continue;
            }
            std::string msgTypeStr = iface["publisher"].as<std::string>();
            MessageType type = getMessageType(msgTypeStr);
            if (type == MessageType::UNKNOWN) continue;
            cfg.msgType = type;
            if (!iface["name"]) {
                RCLCPP_ERROR(logger_, "No name specified. Skipping.");
                continue;
            } 
            std::string name = iface["name"].as<std::string>();
            cfg.name = name;

            if (!iface["topic"]) {
                cfg.topic = getDefaultPublisherTopic(cfg.msgType, cfg.name);
                RCLCPP_INFO(logger_, "No topic specified. Using default '%s'.", cfg.topic.c_str());
            }
            else {
                std::string topic = iface["topic"].as<std::string>();
                if (topic[0] == '/')
                {
                    cfg.topic = topic.substr(1);
                }
                else {
                    cfg.topic = getDefaultPublisherTopic(cfg.msgType, cfg.name);
                    RCLCPP_ERROR(logger_, "Invalid topic name: %s. Must start with /. Using default %s.", topic.c_str(), cfg.topic.c_str());
                }
            }
            publisherConfig_.push_back(cfg);
        }
        if (publisherConfig_.empty()) {
            RCLCPP_ERROR(logger_, "No publishers or only wrong ones specified.");
            return false;
        }

        // Check if transport plugin is defined. Not checked if its true
        if (!params["transport"]["type"] || !params["transport"]["type"].IsScalar())
        {
            RCLCPP_ERROR(logger_, "Invalid or missing transport.type in YAML.");
            return false;
        }
        transport_plugin_name_ = params["transport"]["type"].as<std::string>();

        // Check if transport params are defined, if not use default
        if (!params["transport"]["params"])
        {
            RCLCPP_INFO(logger_, "No params for transport plugin found.");
        }
        else
        {
            transport_params_ = params["transport"]["params"];
        }
        return true;
    }

    MessageType ConfigParser::getMessageType(std::string message_type) {
        if (message_type == "JointTrajectoryController") {
            return MessageType::JOINT_TRAJECTORY_CONTROLLER;
        }
        if (message_type == "JointGroupEffortController") {
            return MessageType::JOINT_GROUP_EFFORT_CONTROLLER;
        }
        if (message_type == "JointGroupPositionController") {
            return MessageType::JOINT_GROUP_POSITION_CONTROLLER;
        }
        if (message_type == "JointGroupVelocityController") {
            return MessageType::JOINT_GROUP_VELOCITY_CONTROLLER;
        }
        RCLCPP_ERROR(logger_, "Controller type '%s' is unknown. Skipping.", message_type.c_str());
        return MessageType::UNKNOWN;
    }

    std::string ConfigParser::getTopicValue(const YAML::Node &params, const std::string &key, const std::string &default_value)
    {
        if (!params[key])
        {
            RCLCPP_INFO(logger_, "No states_topic found. Using default %s.", default_value.c_str());
            return default_value;
        }
        std::string topic = params[key].as<std::string>();
        if (topic[0] == '/')
        {
            return topic.substr(1);
        }
        RCLCPP_ERROR(logger_, "Invalid topic name: %s. Using default %s.", topic.c_str(), default_value.c_str());
        return default_value;
    }

    std::string ConfigParser::getDefaultPublisherTopic(MessageType type, std::string name) {
        TopicMap tm;
        std::string defaultTopic = tm.DEFAULT_TOPIC_MAP[type];
        replace(defaultTopic, "<name>", name);
        return defaultTopic;
    }

    bool ConfigParser::replace(std::string& str, const std::string& from, const std::string& to) {
        size_t start_pos = str.find(from);
        if(start_pos == std::string::npos)
            return false;
        str.replace(start_pos, from.length(), to);
        return true;
    }
} // namespace config
} // namespace ros2_api
