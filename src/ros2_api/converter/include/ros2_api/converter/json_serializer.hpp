#ifndef JSON_SERIALIZER_HPP
#define JSON_SERIALIZER_HPP 


#include <ros2_api/converter/json_serializer_msgs.hpp>
#include <ros2_api_msgs/msg/calculated_states.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nlohmann/json.hpp>
#include <ros2_api/types/types.hpp>

namespace ros2_api {
    namespace converter {
        using json = nlohmann::json;
        class JsonSerializer {
            public:
                static std::vector<std::uint8_t> serialize(ros2_api_msgs::msg::ClientFeedback msg);

                static std::vector<std::uint8_t> serialize(ros2_api_msgs::msg::CalculatedStates msg);
                static std::vector<std::uint8_t> serialize(sensor_msgs::msg::JointState msg);


                static std::pair<std::string, const void*> deserialize(const std::uint8_t* data, int size);
            private:
                static const void* getMessageContent(int type, json payload);
        };
    } // namespace converter
} // namespace ros2_api
#endif //JSON_SERIALIZER_HPP