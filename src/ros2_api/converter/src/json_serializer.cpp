#include <ros2_api/converter/json_serializer.hpp>
#include <ros2_api/types/types.hpp>
#include <nlohmann/json.hpp>


namespace ros2_api {
    namespace converter {

        std::vector<std::uint8_t> JsonSerializer::serialize(ros2_api_msgs::msg::ClientFeedback msg) {
            Feedback f;
            f.feedback = msg;
            json j = f;
            json wrapped = json{
                {"type", MessageType::CLIENT_FEEDBACK},
                {"name", "feedback_channel"},
                {"payload", j}
            };
            return json::to_msgpack(wrapped);
        }

        std::vector<std::uint8_t> JsonSerializer::serialize(ros2_api_msgs::msg::CalculatedStates msg) {
            CalculatedStates calc;
            calc.states = msg;
            json j = calc;
            json wrapped = json{
                {"type", MessageType::CALCULATED_STATES},
                {"name", "calculated_states"},
                {"payload", j}
            };
            return json::to_msgpack(wrapped);
        }

        std::vector<std::uint8_t> JsonSerializer::serialize(sensor_msgs::msg::JointState msg) {
            JointStates js;
            js.states = msg;
            json j = js;
            json wrapped = json{
                {"type", MessageType::JOINT_STATES},
                {"name", "joint_states"},
                {"payload", j}
            };
            return json::to_msgpack(wrapped);
        }

        std::pair<std::string, const void*> JsonSerializer::deserialize(const std::uint8_t* data, int size) {
            std::vector<std::uint8_t> vec(data, data + size);            
            json jsonStructure = json::from_msgpack(vec);
            int type = jsonStructure["type"];
            std::string name = jsonStructure["publisher_name"];
            const void* msg = getMessageContent(type, jsonStructure["payload"]);
            return std::make_pair(name, msg);
        }

        const void* JsonSerializer::getMessageContent(int type, json payload) {
            if (type == MessageType::JOINT_TRAJECTORY_CONTROLLER) {
                JointTrajectory* j = new JointTrajectory(payload);
                return (void*) j;
            }
            if (type == MessageType::JOINT_GROUP_POSITION_CONTROLLER) {
                JointGroupController* j = new JointGroupController(payload);
                return (void*) j;
            }
            if (type == MessageType::JOINT_GROUP_EFFORT_CONTROLLER) {
                JointGroupController* j = new JointGroupController(payload);
                return (void*) j;
            }
            if (type == MessageType::JOINT_GROUP_VELOCITY_CONTROLLER) {
                JointGroupController* j = new JointGroupController(payload);
                return (void*) j;
            }
            if (type == MessageType::CALCULATED_STATES) {
                CalculatedStates* c = new CalculatedStates(payload);
                return (void*) c;
            }
            if (type == MessageType::CLIENT_FEEDBACK) {
                Feedback* f = new Feedback(payload);
                return (void*) f;
            }
            throw std::runtime_error("MessageType unknown");
        }

    } // namespace converter
} // namespace ros2_api