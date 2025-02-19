#ifndef FEEDBACK_SENDER_HPP
#define FEEDBACK_SENDER_HPP

#include <memory>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_api
{
    namespace core
    {
        /**
         * @brief Class responsible for subscribing to the feedback stream and sending them to the clients.
         */
        class FeedbackSender : public rclcpp::Node
        {
        public:
            /**
             * @brief Construct a new FeedbackSender object.
             * 
             * @param communication_protocol Shared pointer to the communication protocol.
             */
            FeedbackSender(std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol);

            /**
             * @brief Destroy the FeedbackSender object.
             */
            ~FeedbackSender() = default;

            /**
             * @brief Send feedback message to the client.
             * 
             * @param feedback The feedback message to be sent.
             */
            void send_feedback(const ros2_api_msgs::msg::ClientFeedback &feedback);

            /**
             * @brief Set up the subscription to the feedback channel.
             */
            void set_up_subscription();

        private:
            std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol_;
            rclcpp::Subscription<ros2_api_msgs::msg::ClientFeedback>::SharedPtr subscriber_feedback_;
        };
    } // namespace core
} // namespace ros2_api

#endif // FEEDBACK_SENDER_HPP