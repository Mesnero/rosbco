#include "ros2_api/config_manager.hpp"
#include <iostream>

namespace robot_api
{
    ConfigManager::ConfigManager()
        : logger_(rclcpp::get_logger("ConfigManager"))
    {}

    ConfigManager &ConfigManager::instance()
    {
        static ConfigManager instance;
        return instance;
    }

    bool ConfigManager::load(const std::string &config_path)
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

    void ConfigManager::reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        config_path_ = "";
        root_node_ = YAML::Node();
        command_interfaces_.clear();
        transport_plugin_name_ = "";
        transport_params_ = YAML::Node();
        state_topic_ = "";
        velocity_topic_ = "";
        position_topic_ = "";
        effort_topic_ = "";
    }

    std::string ConfigManager::getTransportType() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return transport_plugin_name_;
    }

    std::vector<std::string> ConfigManager::getCommandInterfaces() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return command_interfaces_;
    }

    std::string ConfigManager::ConfigManager::getStateTopic() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_topic_;
    }

    std::string ConfigManager::getVelocityTopic() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return velocity_topic_;
    }

    std::string ConfigManager::getPositionTopic() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return position_topic_;
    }

    std::string ConfigManager::getEffortTopic() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return effort_topic_;
    }

    YAML::Node ConfigManager::getTransportParams() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return transport_params_;
    }

    bool ConfigManager::validate_config()
    {
        // Check if the root node has the necessary keys
        if (!root_node_["ros2_api"] || !root_node_["ros2_api"]["ros__parameters"])
        {
            RCLCPP_ERROR(logger_, "Missing 'ros2_api' or 'ros__parameters' in YAML.");
            return false;
        }
        YAML::Node params = root_node_["ros2_api"]["ros__parameters"];

        // Check if the state topic exists, if not use default
        state_topic_ = getTopicValue(params, "joints_topic", "/calc_joint_states");

        // Check if command interfaces are defined and an array
        if (!params["command_interfaces"] || !params["command_interfaces"]["interfaces"].IsSequence())
        {
            RCLCPP_ERROR(logger_, "Invalid or missing command_interfaces.interfaces in YAML.");
            return false;
        }

        for (const auto &iface : params["command_interfaces"]["interfaces"])
        {
            std::string iface_str = iface.as<std::string>();
            if (iface_str != "velocity" && iface_str != "position" && iface_str != "effort")
            {
                RCLCPP_ERROR(logger_, "Invalid command interface: %s", iface_str.c_str());
                return false;
            }
            command_interfaces_.push_back(iface_str);
        }

        // Check if command topics are defined and only add them if they are in the command_interfaces
        velocity_topic_ = getCommandTopic(params, "velocity_topic", "/command_velocity");
        position_topic_ = getCommandTopic(params, "position_topic", "/command_position");
        effort_topic_ = getCommandTopic(params, "effort_topic", "/command_effort");

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

    std::string ConfigManager::getTopicValue(const YAML::Node &params, const std::string &key, const std::string &default_value)
    {
        if (!params[key])
        {
            RCLCPP_INFO(logger_, "No joints_topic found. Using default %s.", default_value.c_str());
            return default_value;
        }
        std::string topic = params[key].as<std::string>();
        if (topic[0] == '/')
        {
            return topic;
        }
        RCLCPP_ERROR(logger_, "Invalid topic name: %s. Using default %s.", topic.c_str(), default_value.c_str());
        return default_value;
    }

    std::string ConfigManager::getCommandTopic(const YAML::Node &params, const std::string &key, const std::string &default_value)
    {
        std::string interface = key.substr(0, key.find("_topic"));
        bool cmd_interface_exists = std::find(command_interfaces_.begin(), command_interfaces_.end(), interface) != command_interfaces_.end();
        if (cmd_interface_exists)
        {
            if (!params["command_interfaces"][key])
            {
                RCLCPP_INFO(logger_, "No topic for %s set. Using default.", interface.c_str());
                return default_value;
            }
            std::string topic = params["command_interfaces"][key].as<std::string>();
            if (topic[0] == '/')
            {
                return topic;
            }
            RCLCPP_ERROR(logger_, "Invalid topic name: %s. Using default %s.", topic.c_str(), default_value.c_str());
            return default_value;
        }
        if (params["command_interfaces"][key])
        {
            RCLCPP_ERROR(logger_, "%s is not a command interface. No value will be set.", interface.c_str());
        }
        return "";
    }
} // namespace robot_api
