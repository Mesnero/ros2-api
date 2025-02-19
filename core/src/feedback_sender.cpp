#include <ros2_api/core/feedback_sender.hpp>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>
#include <ros2_api/converter/json_serializer.hpp>

namespace ros2_api
{
    namespace core
    {
        FeedbackSender::FeedbackSender(std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol)
            : Node("feedback_sender"),
              communication_protocol_(std::move(communication_protocol))
        {
        }

        void FeedbackSender::send_feedback(const ros2_api_msgs::msg::ClientFeedback &feedback)
        {
            converter::JsonSerializer msg_converter;
            std::vector<std::uint8_t> feedback_byte_array;
            try
            {
                feedback_byte_array = msg_converter.serialize(feedback);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error serializing message.");
                return;
            }
            communication_protocol_->send_to_client(feedback_byte_array.data(), feedback_byte_array.size());
        }

        void FeedbackSender::set_up_subscription()
        {
            subscriber_feedback_ = this->create_subscription<ros2_api_msgs::msg::ClientFeedback>(
                "feedback_channel",
                10,
                [this](const ros2_api_msgs::msg::ClientFeedback::SharedPtr msg)
                {
                    send_feedback(*msg);
                });
        }

    } // namespace core
} // namespace ros2_api