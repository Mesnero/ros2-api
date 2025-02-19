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
        /**
         * @brief Class responsible for subscribing to state topics and sending the states to clients.
         */
        class StateSender : public rclcpp::Node
        {
        public:
            /**
             * @brief Construct a new StateSender object.
             * 
             * @param communication_protocol Shared pointer to the communication protocol.
             */
            StateSender(std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol);

            /**
             * @brief Destroy the StateSender object.
             */
            ~StateSender() = default;

            /**
             * @brief Set up the subscription to the state topic.
             */
            void set_up_subscription();

        private:
            /**
             * @brief Send calculated state message to the client.
             * 
             * @param state The calculated state message to be sent.
             */
            void send_calculated_state(const ros2_api_msgs::msg::CalculatedStates &state);

            /**
             * @brief Send joint state message to the client.
             * 
             * @param state The joint state message to be sent.
             */
            void send_joint_state(const sensor_msgs::msg::JointState &state);

            std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol_;
            rclcpp::Subscription<ros2_api_msgs::msg::CalculatedStates>::SharedPtr subscriber_calc_state_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_joint_state_;
            std::string state_topic_;
            bool use_calculated_states_;
        };
    } // namespace core
} // namespace ros2_api
#endif // STATE_SENDER_HPP