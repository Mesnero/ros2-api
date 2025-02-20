#ifndef CORE_API_TYPES_HPP
#define CORE_API_TYPES_HPP

#include <string>
#include <unordered_map>


/**
 * @file types.hpp
 * @brief This file contains the definitions for message types, message type mapping, and feedback codes used in the ros2_api namespace.
 */
namespace ros2_api
{
	/**
	 * @enum MessageType
	 * @brief Enum representing different types of messages.
	 */
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

	/**
	 * @class MessageTypeMapper
	 * @brief Class for mapping message types to their corresponding default topics.
	 */
	class MessageTypeMapper
	{
	public:
		/**
		 * @brief Constructor that initializes the message type to topic mapping.
		 */
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

		/**
		 * @brief Get the MessageType from a string key.
		 * @param key The string key representing the message type.
		 * @return The corresponding MessageType.
		 */
		MessageType getTypeFromString(const std::string &key) const
		{
			auto it = mapping_.find(key);
			if (it != mapping_.end())
			{
				return it->second.first;
			}
			return UNKNOWN;
		}

		/**
		 * @brief Get the default topic for a given message type key and name.
		 * @param key The string key representing the message type.
		 * @param name The name to replace in the topic string.
		 * @return The default topic string.
		 */
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

		/**
		 * @brief Get the default topic for a given MessageType and name.
		 * @param type The MessageType.
		 * @param name The name to replace in the topic string.
		 * @return The default topic string.
		 */
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
		/**
		 * @brief Replace a substring in a string with another string.
		 * @param str The original string.
		 * @param from The substring to be replaced.
		 * @param to The string to replace with.
		 * @return True if the replacement was successful, false otherwise.
		 */
		static bool replace(std::string &str, const std::string &from, const std::string &to)
		{
			size_t start_pos = str.find(from);
			if (start_pos == std::string::npos)
				return false;
			str.replace(start_pos, from.length(), to);
			return true;
		}

		/**
		 * @brief Mapping from string keys to pairs of MessageType and default topic strings.
		 */
		std::unordered_map<std::string, std::pair<MessageType, std::string>> mapping_;
	};

	/**
	 * @struct MessageConfig
	 * @brief Struct representing the configuration of a message.
	 */
	struct MessageConfig
	{
		MessageType msg_type;
		std::string name;
		std::string topic;
	};

	/**
	 * @enum FeedbackCode
	 * @brief Enum representing different feedback codes.
	 */
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
