#include <ros2_api/communication_protocol.hpp>
#include <ros2_api/communication_manager.hpp>
#include <ros2_api/message_sender.hpp>
#include <ros2_api/message_handler.hpp>
#include <ros2_api/unix_domain_socket_protocol.hpp>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

namespace robot_api
{
    CommunicationManager::CommunicationManager(std::shared_ptr<rclcpp::Node> node) : node_(node)
    {
        messageHandler_ = std::make_unique<MessageHandler>(this);
        messageSender_ = std::make_unique<MessageSender>(this);
    }

    CommunicationManager::~CommunicationManager() {}

    void CommunicationManager::setCommunicationProtocol(std::string msgProtocol)
    {
        // TODO: Make the input a ICommunicationProtocol
        if (msgProtocol == "IPC")
        {
            messageProtocol_ = std::make_unique<UnixDomainSocketProtocol>(node_->get_logger());
        }
        messageProtocol_->subscribe(*this);
    }

    void CommunicationManager::startServer()
    {
        if (messageProtocol_ != nullptr)
        {
            RCLCPP_INFO(node_->get_logger(), "Starting server");
            messageProtocol_->start();
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "No message protocol set");
        }
    }

    // MessageHandler or MessageSender calls this function if they want to talk to client
    void CommunicationManager::sendMessage(const char *message, int length)
    {
        if (messageProtocol_ != nullptr)
        {
            messageProtocol_->send(message, length);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "No message protocol set");
        }
    }

    // CommunicationProtocol calls this function if it receives a message
    void CommunicationManager::receiveMessage(const char *message, int length)
    {
        messageHandler_->handleMessage(message, length);
    }

    std::shared_ptr<rclcpp::Node> CommunicationManager::getNode()
    {
        return node_;
    }

} // namespace robot_api