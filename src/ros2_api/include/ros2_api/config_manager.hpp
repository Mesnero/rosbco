#ifndef CONFIG_MANAGER_HPP
#define CONFIG_MANAGER_HPP

#include <yaml-cpp/yaml.h>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace robot_api {

class ConfigManager {
public:
    // Singleton Access
    static ConfigManager& instance();
    
    // Public interface
    bool load(const std::string& config_path);
    std::string getTransportType() const;
    std::vector<std::string> getCommandInterfaces() const;
    std::string getStateTopic() const;
    std::string getVelocityTopic() const;
    std::string getPositionTopic() const;
    std::string getEffortTopic() const;
    YAML::Node getTransportParams() const;
    void reset();

private:
    // Private constructor/destructor
    ConfigManager();
    ~ConfigManager() = default;

    // Delete copy/move operations
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    ConfigManager(ConfigManager&&) = delete;
    ConfigManager& operator=(ConfigManager&&) = delete;

    // Validation methods
    bool validate_config();
    std::string getTopicValue(const YAML::Node& params, 
                            const std::string& key,
                            const std::string& default_value);
    std::string getCommandTopic(const YAML::Node& params,
                              const std::string& key,
                              const std::string& default_value);

    // Data members
    mutable std::mutex mutex_;
    YAML::Node root_node_;
    std::string config_path_;
    rclcpp::Logger logger_;
    std::vector<std::string> command_interfaces_;
    std::string transport_plugin_name_;
    YAML::Node transport_params_;
    std::string state_topic_;
    std::string velocity_topic_;
    std::string position_topic_;
    std::string effort_topic_;


};

} // namespace robot_api
#endif // CONFIG_MANAGER_HPP