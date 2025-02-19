#ifndef CORE_API_TYPES_HPP
#define CORE_API_TYPES_HPP

#include <string>
#include <unordered_map>
#include <functional>
#include <memory>

namespace ros2_api
{

	enum MessageType
	{
		UNKNOWN = 0,
		CLIENT_FEEDBACK = 1,
		JOINT_STATES = 2,
		CALCULATED_STATES = 10,
		JOINT_TRAJECTORY_CONTROLLER = 20,
		JOINT_GROUP_POSITION_CONTROLLER = 30,
		JOINT_GROUP_EFFORT_CONTROLLER = 31,
		JOINT_GROUP_VELOCITY_CONTROLLER = 32,
	};

	class MessageTypeMapper
	{
	public:
		MessageTypeMapper()
		{
			mapping_["JointTrajectoryController"] = {JOINT_TRAJECTORY_CONTROLLER, "<name>/joint_trajectory"};
			mapping_["JointGroupPositionController"] = {JOINT_GROUP_POSITION_CONTROLLER, "<name>/commands"};
			mapping_["JointGroupEffortController"] = {JOINT_GROUP_EFFORT_CONTROLLER, "<name>/commands"};
			mapping_["JointGroupVelocityController"] = {JOINT_GROUP_VELOCITY_CONTROLLER, "<name>/commands"};
			mapping_["CalculatedStates"] = {CALCULATED_STATES, "calc_joint_states"};
			mapping_["ClientFeedback"] = {CLIENT_FEEDBACK, "feedback_channel"};
			mapping_["JointStates"] = {JOINT_STATES, "joint_states"};
		}

		MessageType getTypeFromString(const std::string &key) const
		{
			auto it = mapping_.find(key);
			if (it != mapping_.end())
			{
				return it->second.first;
			}
			return UNKNOWN;
		}

		std::string getDefaultTopic(const std::string &key, const std::string &name) const
		{
			auto it = mapping_.find(key);
			if (it != mapping_.end())
			{
				std::string topic = it->second.second;
				replace(topic, "<name>", name);
				return topic;
			}
			return "";
		}

		std::string getDefaultTopic(MessageType type, const std::string &name) const
		{
			for (const auto &entry : mapping_)
			{
				if (entry.second.first == type)
				{
					std::string topic = entry.second.second;
					replace(topic, "<name>", name);
					return topic;
				}
			}
			return "";
		}

	private:
		static bool replace(std::string &str, const std::string &from, const std::string &to)
		{
			size_t start_pos = str.find(from);
			if (start_pos == std::string::npos)
				return false;
			str.replace(start_pos, from.length(), to);
			return true;
		}

		std::unordered_map<std::string, std::pair<MessageType, std::string>> mapping_;
	};

	struct MessageConfig
	{
		MessageType msgType;
		std::string name;
		std::string topic;
	};

	enum FeedbackCode
	{
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
