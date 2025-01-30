#ifndef MESSAGE_SENDER_HPP
#define MESSAGE_SENDER_HPP

#include <memory>
#include <ros2_api/communication_manager.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot_api
{
    class MessageSender
    {
    public:
        MessageSender(CommunicationManager *communication_manager);
        ~MessageSender() = default;

    private:
        CommunicationManager *communication_manager_;
        std::shared_ptr<rclcpp::Node> node_;
    };

} // namespace robot_api
#endif // MESSAGE_SENDER_HPP