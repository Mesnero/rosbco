#include <ros2_api/message_handler.hpp>
#include <ros2_api/communication_manager.hpp>

namespace robot_api
{
    MessageHandler::MessageHandler(CommunicationManager *communication_manager)
        : communication_manager_(communication_manager)
    {
        node_ = communication_manager_->getNode();
    }

    void MessageHandler::handleMessage(const char *message, int length)
    {
        std::printf("MessageHandler received message: %s\n", message);
    }

} // namespace robot_api