#ifndef COMMUNICATION_MANAGER_HPP
#define COMMUNICATION_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>


namespace robot_api
{
    class MessageSender;  
    class MessageHandler;
    class CommunicationProtocol;

    class CommunicationManager
    {
    public:
        //TODO: Do i need two constructors?
        CommunicationManager();
        CommunicationManager(std::shared_ptr<rclcpp::Node> node);
        ~CommunicationManager();
        // TODO: Make the input a ICommunicationProtocol
        void setCommunicationProtocol(const std::string msgProtocol);
        void startServer();
        void sendMessage(const char *message, int length);
        void receiveMessage(const char *message, int length);
        std::shared_ptr<rclcpp::Node> getNode();

    private:
        
        std::unique_ptr<CommunicationProtocol> messageProtocol_;
        std::unique_ptr<MessageSender> messageSender_;
        std::unique_ptr<MessageHandler> messageHandler_; 
        std::shared_ptr<rclcpp::Node> node_;
    };

} // namespace robot_api
#endif // COMMUNICATION_MANAGER_HPP