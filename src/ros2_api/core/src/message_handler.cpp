#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>

#include <ros2_api/core/message_handler.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <ros2_api/types/types.hpp>

#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api/publisher/publisher_factory.hpp>
#include <ros2_api/converter/json_serializer.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>


namespace ros2_api
{
    using Message = const void*; 
    namespace core
    {
        MessageHandler::MessageHandler(
            std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol
        ): Node("message_handler"), communicationProtocol_{communicationProtocol}
        {
            communicationProtocol_->setCallback(
                [this](const std::uint8_t *message, int length) { this->handleMessage(message, length); }
            );
        }

        void MessageHandler::setUpPublishers(std::vector<MessageConfig> publisherConfig) {
            for(auto &config : publisherConfig) {
                if(publisherMap_.find(config.name) != publisherMap_.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Publisher with name '%s' already added.", config.name.c_str());
                }
                publisherMap_[config.name] = publisher::PublisherFactory::createPublisher(shared_from_this(), config.msgType, config.topic, 10);
            }
            publisherFeedback_ = this->create_publisher<ros2_api_msgs::msg::ClientFeedback>("feedback_channel", 10);
        }

        void MessageHandler::handleMessage(const std::uint8_t *message, int length) {
            converter::JsonSerializer msgConverter;
            std::pair<std::string, Message> publisherMsg;
            try {
                publisherMsg = msgConverter.deserialize(message, length);
            }
            catch(const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Error decoding: %s", e.what());
                ros2_api_msgs::msg::ClientFeedback feedback;
                feedback.message = "Error when deserializing message: " + std::string(e.what());
                feedback.status_code = FeedbackCode::UNEXPECTED_MSG_STRUCTURE;
                publisherFeedback_.get()->publish(feedback);
                return;
            }
            std::string name = publisherMsg.first;
            Message messageToPublish = publisherMsg.second;
            auto it = publisherMap_.find(name);
            if (it != publisherMap_.end()) {
                std::unique_ptr<IMessagePublisher>& publisher = it->second;
                publisher.get()->publish(messageToPublish);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Publisher with name '%s' not found.", name.c_str());
                ros2_api_msgs::msg::ClientFeedback feedback;
                feedback.message = "Publisher with name '" + name + "' not found.";
                feedback.status_code = FeedbackCode::PUBLISHER_NOT_FOUND;
                publisherFeedback_.get()->publish(feedback);
            }
        }
    } // namespace core
} // namespace ros2_api
