#ifndef FEEDBACK_SENDER_HPP
#define FEEDBACK_SENDER_HPP

#include <memory>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_api
{
    namespace core
    {
        class FeedbackSender : public rclcpp::Node
        {
        public:
            FeedbackSender(std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol);
            ~FeedbackSender() = default;

            void sendFeedback(const ros2_api_msgs::msg::ClientFeedback &feedback);
            void setUpSubscription();
        private:

            std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol_;
            rclcpp::Subscription<ros2_api_msgs::msg::ClientFeedback>::SharedPtr subscriberFeedback_;
        };
    } // namespace core
} // namespace ros2_api

#endif // FEEDBACK_SENDER_HPP