#ifndef CORE_API_TYPES_HPP
#define CORE_API_TYPES_HPP

#include <string>
#include <unordered_map>

namespace ros2_api {

enum MessageType {
	JOINT_TRAJECTORY_CONTROLLER = 20,
	JOINT_GROUP_POSITION_CONTROLLER = 30,
	JOINT_GROUP_EFFORT_CONTROLLER = 31,
	JOINT_GROUP_VELOCITY_CONTROLLER = 32,
	CALCULATED_STATES = 10,
	JOINT_STATES = 2,
	CLIENT_FEEDBACK = 1,
	UNKNOWN = 0
};

//TODO: CHANGE!
struct TopicMap {
	std::unordered_map<MessageType, std::string> DEFAULT_TOPIC_MAP = {
		std::make_pair<MessageType, std::string>(JOINT_TRAJECTORY_CONTROLLER, "<name>/joint_trajectory"),
		std::make_pair<MessageType, std::string>(JOINT_GROUP_POSITION_CONTROLLER, "<name>/commands"),
		std::make_pair<MessageType, std::string>(JOINT_GROUP_EFFORT_CONTROLLER, "<name>/commands"),
		std::make_pair<MessageType, std::string>(JOINT_GROUP_VELOCITY_CONTROLLER, "<name>/commands"),
		std::make_pair<MessageType, std::string>(CALCULATED_STATES, "calc_joint_states"),
		std::make_pair<MessageType, std::string>(CLIENT_FEEDBACK, "feedback_channel"),
		std::make_pair<MessageType, std::string>(JOINT_STATES, "joint_states")
	};
};

struct MessageConfig {
	MessageType msgType;
	std::string name;
	std::string topic;
};

enum FeedbackCode {
	PUBLISHER_NOT_FOUND,
	UNEXPECTED_MSG_STRUCTURE,
	API_BLOCKED,
	ROBOT_DISCONNECTED_UNEXPECTEDLY,
	ROBOT_BREAKS,
	WORKSPACE_VIOLATION,
	SELF_COLLISION,
	COLLISION_WITH_COLLISION_OBJECT,
	POSITION_VIOLATION,
	JERK_VIOLATION,
	ACCELERATION_VIOLATION,
	VELOCITY_VIOLATION,
	WRONG_AMOUNT_OF_JOINTS,
	COMMAND_VALIDATED_SUCCESSFULLY,
	COMMAND_EXECUTED
};

} // namespace ros2_api

#endif // CORE_API_TYPES_HPP