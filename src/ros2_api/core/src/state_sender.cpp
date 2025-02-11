#include <ros2_api/core/state_sender.hpp>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api_msgs/msg/calculated_states.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <ros2_api/converter/json_serializer.hpp>
#include <string>

namespace ros2_api
{
    namespace core
    {
        StateSender::StateSender(std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol)
        : Node("state_sender"), 
        communicationProtocol_(std::move(communicationProtocol))
        {
            stateTopic_ = ros2_api::config::ConfigParser::instance().getStateTopic();
            use_calculated_states_ = ros2_api::config::ConfigParser::instance().useCalculatedStates();
        }

        void StateSender::sendCalculatedState(const ros2_api_msgs::msg::CalculatedStates &state)
        {
            converter::JsonSerializer msgConverter; 
            std::vector<std::uint8_t> stateByteArray = msgConverter.serialize(state);
            communicationProtocol_->sendToClient(stateByteArray.data(), stateByteArray.size());
        }

        void StateSender::sendJointState(const sensor_msgs::msg::JointState &state) {
            converter::JsonSerializer msgConverter;
            std::vector<std::uint8_t> stateByteArray = msgConverter.serialize(state);
            communicationProtocol_->sendToClient(stateByteArray.data(), stateByteArray.size());            
        }

        void StateSender::setUpSubscription() 
        {   
            if(use_calculated_states_) {
                subscriberCalcStates_ = this->create_subscription<ros2_api_msgs::msg::CalculatedStates>(
                    stateTopic_,
                    10,
                    [this](const ros2_api_msgs::msg::CalculatedStates::SharedPtr msg) {
                    sendCalculatedState(*msg);
                });
            } else {
                subscriberJointStates_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    stateTopic_,
                    10,
                    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                        sendJointState(*msg);
                    }
                );
            }

        }

    } // namespace core
} // namespace ros2