#ifndef MESSAGE_HANDLER_HPP
#define MESSAGE_HANDLER_HPP

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <ros2_api/protocol_base/communication_protocol.hpp>

#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ros2_api/types/types.hpp>
#include <ros2_api/publisher/message_publisher_interface.hpp>

using TrajectoryPublisher = rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr;
using GroupPublisher = rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;

namespace ros2_api
{
    namespace core
    {
        /**
         * @brief Class responsible for handling incoming messages and publishing them to the appropriate ROS2 topics.
         */
        class MessageHandler : public rclcpp::Node
        {
        public:
            /**
             * @brief Construct a new MessageHandler object.
             * 
             * @param communication_protocol Shared pointer to the communication protocol.
             */
            MessageHandler(std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol);

            /**
             * @brief Destroy the MessageHandler object.
             */
            ~MessageHandler() = default;

            /**
             * @brief Set up publishers based on the provided configuration.
             * 
             * @param publisher_config Vector of publisher configurations.
             */
            void set_up_publishers(std::vector<MessageConfig> publisher_config);

            /**
             * @brief Handle incoming messages and publish them to the appropriate topics.
             * 
             * @param message Pointer to the message data.
             * @param length Length of the message data.
             */
            void handle_message(const std::uint8_t *message, int length);

        private:
            std::unordered_map<std::string, std::unique_ptr<IMessagePublisher>> publisher_map_;

            rclcpp::Publisher<ros2_api_msgs::msg::ClientFeedback>::SharedPtr publisher_feedback_;
            std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol_;
        };
    } // namespace core
} // namespace ros2_api

#endif // MESSAGE_HANDLER_HPP