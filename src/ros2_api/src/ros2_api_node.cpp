#ifndef ROS2_API_NODE_HPP
#define ROS2_API_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <ros2_api/communication_manager.hpp>
#include <ros2_api/config_manager.hpp> 



namespace robot_api
{
    const std::string PATH_TO_YAML = "/";

    class Ros2ApiServerNode : public rclcpp::Node
    {
    public:
        Ros2ApiServerNode() : Node("server_node")
        {
            this->declare_parameter<std::string>("path_to_yaml", PATH_TO_YAML);
            std::string pathToYaml = this->get_parameter("path_to_yaml").as_string();
            ConfigManager::instance().load(pathToYaml);
        }

        void initialize() {
            /* communication_manager_ = std::make_shared<CommunicationManager>(shared_from_this());
            communication_manager_->setCommunicationProtocol(protocol_);
            communication_manager_->startServer();
            this->create_publisher<std::string>("vel_cmd", 10); */
        }

    private:
        // TODO: LATER ON CONFIGURED BY CONFIG AND MADE REAL PROTOCOL
        /* std::shared_ptr<CommunicationManager> communication_manager_; */
    };

} // namespace robot_api
#endif // ROS2_API_NODE_HPP

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_api::Ros2ApiServerNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
