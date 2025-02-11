#include <ros2_api/core/feedback_sender.hpp>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <ros2_api/converter/json_serializer.hpp>

namespace ros2_api
{
namespace core
{
    FeedbackSender::FeedbackSender(std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol)
    : Node("feedback_sender"),
      communicationProtocol_(std::move(communicationProtocol))
    {} 

    void FeedbackSender::sendFeedback(const ros2_api_msgs::msg::ClientFeedback &feedback)
    {
        converter::JsonSerializer msgConverter;
        std::vector<std::uint8_t> feedbackByteArray;
        try {
            feedbackByteArray = msgConverter.serialize(feedback);
        } catch(const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error serializing message.");
            return;
        }
        communicationProtocol_->sendToClient(feedbackByteArray.data(), feedbackByteArray.size());
    }

    void FeedbackSender::setUpSubscription() {
        subscriberFeedback_ = this->create_subscription<ros2_api_msgs::msg::ClientFeedback>(
                "feedback_channel",
                10,
                [this](const ros2_api_msgs::msg::ClientFeedback::SharedPtr msg) {
                    sendFeedback(*msg);
        });
    }

} // namespace core
} // namespace ros2_api