#include <string>
#include <sys/stat.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_api/core/feedback_sender.hpp>
#include <ros2_api/core/message_handler.hpp>
#include <ros2_api/core/state_sender.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros2_api/protocol_base/communication_protocol.hpp>

std::string parseConfigFilePath(int argc, char** argv)
{
  std::string path;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config_file" && (i + 1 < argc)) {
      path = argv[i + 1];
      break;
    }
  }
  return path;
}

bool fileExists(const std::string& path) {
  struct stat buffer;
  return (stat(path.c_str(), &buffer) == 0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string config_path = parseConfigFilePath(argc, argv);

    if (config_path.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("LoadYaml"), "No --config_file specified!");
        rclcpp::shutdown();
    }

    if (!fileExists(config_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("LoadYaml"), "Yaml file doesn't exist.");
        rclcpp::shutdown();
    }

    ros2_api::config::ConfigParser::instance().load(config_path);
    std::string protocolType = ros2_api::config::ConfigParser::instance().getTransportType();
    
    std::shared_ptr<protocol_base::CommunicationProtocol> protocol;
    pluginlib::ClassLoader<protocol_base::CommunicationProtocol> comm_loader("protocol_base", "protocol_base::CommunicationProtocol");
    try {
        protocol = comm_loader.createSharedInstance(protocolType);
    }
    catch(pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("PluginCreator"), "Error creating plugin %s. Error was %s", protocolType.c_str(), ex.what());
        rclcpp::shutdown();
    }
    auto protocolParams = ros2_api::config::ConfigParser::instance().getTransportParams();
    protocol->initialize(protocolParams);

    auto publisherConfig = ros2_api::config::ConfigParser::instance().getPublisherConfig();
    auto message_handler_node = std::make_shared<ros2_api::core::MessageHandler>(protocol);
    message_handler_node.get()->setUpPublishers(publisherConfig);
    auto feedback_sender_node = std::make_shared<ros2_api::core::FeedbackSender>(protocol);
    feedback_sender_node.get()->setUpSubscription();
    auto state_sender_node = std::make_shared<ros2_api::core::StateSender>(protocol);
    state_sender_node.get()->setUpSubscription();



    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(message_handler_node);
    executor.add_node(state_sender_node);
    executor.add_node(feedback_sender_node);
    protocol->start();
    executor.spin();

    protocol->stop();
    rclcpp::shutdown();
    return 0;
}