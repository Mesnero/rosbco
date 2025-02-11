#ifndef MESSAGE_HANDLER_HPP
#define MESSAGE_HANDLER_HPP

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <ros2_api/protocol_base/communication_protocol.hpp>

#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ros2_api/types/types.hpp>
#include <ros2_api/publisher/message_publisher_interface.hpp>

using trajectoryPublisher = rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr;
using groupPublisher = rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;

namespace ros2_api
{
    namespace core
    {
        class MessageHandler : public rclcpp::Node
        {
        public:
            MessageHandler(std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol);
            ~MessageHandler() = default;
            void setUpPublishers(std::vector<MessageConfig> publisherConfig);
            void handleMessage(const std::uint8_t *message, int length);
        private:
            std::map<std::string, trajectoryPublisher> trajectoryPublisher_;
            std::map<std::string, groupPublisher> groupPublisher_;
            std::unordered_map<std::string, std::unique_ptr<IMessagePublisher>> publisherMap_;
            
            rclcpp::Publisher<ros2_api_msgs::msg::ClientFeedback>::SharedPtr publisherFeedback_;
            std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol_;
        };
    } // namespace core
} // namespace ros2_api

#endif // MESSAGE_HANDLER_HPP