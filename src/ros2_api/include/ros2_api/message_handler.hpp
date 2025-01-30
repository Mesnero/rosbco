#ifndef MESSAGE_HANDLER_HPP
#define MESSAGE_HANDLER_HPP

#include <memory>
#include <ros2_api/communication_manager.hpp>

namespace robot_api
{
    class MessageHandler
    {
    public:
        //TODO: Maybe make logger optional
        MessageHandler(CommunicationManager *communication_manager);
        ~MessageHandler() = default;
        void handleMessage(const char *message, int length);
    private:
        CommunicationManager *communication_manager_;
        std::shared_ptr<rclcpp::Node> node_;
    };

} // namespace robot_api
#endif // MESSAGE_HANDLER_HPP