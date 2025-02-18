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
	PUBLISHER_NOT_FOUND = 1,
	UNEXPECTED_MSG_STRUCTURE = 2,
	ROBOT_DISCONNECTED_UNEXPECTEDLY = 3,
	ROBOT_BREAKS = 101,
	WORKSPACE_VIOLATION = 102,
	SELF_COLLISION = 103,
	COLLISION_WITH_COLLISION_OBJECT = 104,
	POSITION_VIOLATION = 105,
	JERK_VIOLATION = 106,
	ACCELERATION_VIOLATION = 107,
	VELOCITY_VIOLATION = 108,
	WRONG_AMOUNT_OF_JOINTS = 120,
	COMMAND_VALIDATED = 200,
};

} // namespace ros2_api

#endif // CORE_API_TYPES_HPP