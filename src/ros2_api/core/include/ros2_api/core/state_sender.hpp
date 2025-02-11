#ifndef STATE_SENDER_HPP
#define STATE_SENDER_HPP

#include <memory>
#include <string>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ros2_api_msgs/msg/calculated_states.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_api
{
    namespace core
    {
        class StateSender : public rclcpp::Node
        {
        public:
            StateSender(std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol);
            ~StateSender() = default;

            void setUpSubscription();
        private:
            void sendCalculatedState(const ros2_api_msgs::msg::CalculatedStates &state);
            void sendJointState(const sensor_msgs::msg::JointState &state);
            std::shared_ptr<protocol_base::CommunicationProtocol> communicationProtocol_;
            rclcpp::Subscription<ros2_api_msgs::msg::CalculatedStates>::SharedPtr subscriberCalcStates_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriberJointStates_;
            std::string stateTopic_;
            bool use_calculated_states_;
        };
    } // namespace core
} // namespace ros2_api
#endif // STATE_SENDER_HPP