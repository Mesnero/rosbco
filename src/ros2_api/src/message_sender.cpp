#include <ros2_api/message_sender.hpp>
#include <ros2_api/communication_manager.hpp>

namespace robot_api
{
    MessageSender::MessageSender(CommunicationManager *communication_manager)
        : communication_manager_(communication_manager)
    {
        node_ = communication_manager_->getNode();
    }

} // namespace robot_api