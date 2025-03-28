#include <nlohmann/json.hpp>

#include <ros2_api/converter/json_serializer.hpp>
#include <ros2_api/types/types.hpp>

namespace ros2_api
{
    namespace converter
    {

        std::vector<std::uint8_t> JsonSerializer::serialize(ros2_api_msgs::msg::ClientFeedback msg)
        {
            Feedback f;
            f.feedback = msg;
            json j = f;
            json wrapped = json{
                {"type", MessageType::CLIENT_FEEDBACK},
                {"name_publisher", "feedback_channel"},
                {"payload", j}};
            return json::to_msgpack(wrapped);
        }

        std::vector<std::uint8_t> JsonSerializer::serialize(sensor_msgs::msg::JointState msg)
        {
            JointStates js;
            js.states = msg;
            json j = js;
            json wrapped = json{
                {"type", MessageType::JOINT_STATES},
                {"name_publisher", "joint_states"},
                {"payload", j}};
            return json::to_msgpack(wrapped);
        }

        std::pair<std::string, const void *> JsonSerializer::deserialize(const std::uint8_t *data, int size)
        {
            std::vector<std::uint8_t> vec(data, data + size);
            json json_structure = json::from_msgpack(vec);
            int type;
            json_structure.at("type").get_to(type);
            std::string name;
            json_structure.at("name_publisher").get_to(name);
            json payload;
            json_structure.at("payload").get_to(payload);
            const void *msg = get_message_content(type, payload);
            return std::make_pair(name, msg);
        }

        const void *JsonSerializer::get_message_content(int type, json payload)
        {
            if (type == MessageType::JOINT_TRAJECTORY_CONTROLLER)
            {
                JointTrajectory *j = new JointTrajectory(payload);
                return (void *)j;
            }
            if (type == MessageType::JOINT_GROUP_POSITION_CONTROLLER)
            {
                JointGroupController *j = new JointGroupController(payload);
                return (void *)j;
            }
            if (type == MessageType::JOINT_GROUP_EFFORT_CONTROLLER)
            {
                JointGroupController *j = new JointGroupController(payload);
                return (void *)j;
            }
            if (type == MessageType::JOINT_GROUP_VELOCITY_CONTROLLER)
            {
                JointGroupController *j = new JointGroupController(payload);
                return (void *)j;
            }
            if (type == MessageType::CLIENT_FEEDBACK)
            {
                Feedback *f = new Feedback(payload);
                return (void *)f;
            }
            if (type == MessageType::JOY_MESSAGE) 
            {
                Joy *j = new Joy(payload);
                return (void *)j;
            }
            throw std::runtime_error("MessageType unknown");
        }

    } // namespace converter
} // namespace ros2_api