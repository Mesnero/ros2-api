#ifndef PUBLISHER_FACTORY_HPP
#define PUBLISHER_FACTORY_HPP

#include <memory>
#include <ros2_api/publisher/message_publisher_interface.hpp>
#include <ros2_api/publisher/message_publisher.hpp>
#include <ros2_api/types/types.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ros2_api_msgs/msg/calculated_states.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>

namespace ros2_api
{
    namespace publisher
    {

        /**
         * @class PublisherFactory
         * @brief Factory class for creating IMessagePublisher instances.
         */
        class PublisherFactory
        {
        public:
            /**
             * @brief Creates a publisher based on the specified message type.
             *
             * @param node Shared pointer to the rclcpp::Node.
             * @param type The type of message to be published.
             * @param topic The topic name to publish to.
             * @param queue_size The size of the message queue.
             * @return std::unique_ptr<IMessagePublisher> A unique pointer to the created publisher.
             *
             * @throws std::runtime_error if the MessageType is unknown.
             */
            static std::unique_ptr<IMessagePublisher> create_publisher(
                const rclcpp::Node::SharedPtr &node, ros2_api::MessageType type, std::string topic, size_t queue_size)
            {
                switch (type)
                {
                case MessageType::JOINT_GROUP_POSITION_CONTROLLER:
                case MessageType::JOINT_GROUP_EFFORT_CONTROLLER:
                case MessageType::JOINT_GROUP_VELOCITY_CONTROLLER:
                    return std::make_unique<TypedPublisher<std_msgs::msg::Float64MultiArray>>(node, topic, queue_size);

                case MessageType::JOINT_TRAJECTORY_CONTROLLER:
                    return std::make_unique<TypedPublisher<trajectory_msgs::msg::JointTrajectory>>(node, topic, queue_size);

                case MessageType::CALCULATED_STATES:
                    return std::make_unique<TypedPublisher<ros2_api_msgs::msg::CalculatedStates>>(node, topic, queue_size);

                case MessageType::CLIENT_FEEDBACK:
                    return std::make_unique<TypedPublisher<ros2_api_msgs::msg::ClientFeedback>>(node, topic, queue_size);

                default:
                    throw std::runtime_error("Unknown MessageType");
                }
            }
        };

    } // namespace publisher
} // namespace ros2_api
#endif // PUBLISHER_FACTORY_HPP