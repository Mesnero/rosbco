#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <ros2_api/types/types.hpp>

namespace ros2_api {
namespace config {

class ConfigParser {
public:
    // Singleton Access
    static ConfigParser& instance();
    
    // Public interface
    bool load(const std::string& config_path);
    std::string getTransportType() const;
    std::string getStateTopic() const;
    YAML::Node getTransportParams() const;
    std::vector<std::string> getJointNames() const;
    std::string getBaseFrame() const;
    std::vector<MessageConfig> getPublisherConfig() const;
    bool useCalculatedStates() const;
    void reset();

private:
    // Private constructor/destructor
    ConfigParser();
    ~ConfigParser() = default;

    // Delete copy/move operations
    ConfigParser(const ConfigParser&) = delete;
    ConfigParser& operator=(const ConfigParser&) = delete;
    ConfigParser(ConfigParser&&) = delete;
    ConfigParser& operator=(ConfigParser&&) = delete;

    // Validation methods
    bool validate_config();
    std::string getTopicValue(const YAML::Node& params, 
                            const std::string& key,
                            const std::string& default_value);

    MessageType getMessageType(std::string message_type);
    std::string getDefaultPublisherTopic(MessageType type, std::string name);
    bool replace(std::string& str, const std::string& from, const std::string& to);

    // Data members
    mutable std::mutex mutex_;
    YAML::Node root_node_;
    std::string config_path_;
    rclcpp::Logger logger_;
    std::string base_frame_;
    std::string state_topic_;
    std::string transport_plugin_name_;
    YAML::Node transport_params_;
    bool use_calculated_states_;
    std::vector<std::string> joint_names_;
    std::vector<MessageConfig> publisherConfig_;
};
} // namespace config
} // namespace ros2_api
#endif // CONFIG_PARSER_HPP