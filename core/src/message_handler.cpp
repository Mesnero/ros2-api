#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>

#include <ros2_api/core/message_handler.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <ros2_api/types/types.hpp>

#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api/publisher/publisher_factory.hpp>
#include <ros2_api/converter/json_serializer.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>

namespace ros2_api
{
    using Message = const void *;
    namespace core
    {
        MessageHandler::MessageHandler(
            std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol) : Node("message_handler"), communication_protocol_{communication_protocol}
        {
            communication_protocol_->set_callback(
                [this](const std::uint8_t *message, int length)
                { this->handle_message(message, length); });
        }

        void MessageHandler::set_up_publishers(std::vector<MessageConfig> publisher_config)
        {
            for (auto &config : publisher_config)
            {
                if (publisher_map_.find(config.name) != publisher_map_.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Publisher with name '%s' already added.", config.name.c_str());
                }
                publisher_map_[config.name] = publisher::PublisherFactory::create_publisher(shared_from_this(), config.msg_type, config.topic, 10);
            }
            publisher_feedback_ = this->create_publisher<ros2_api_msgs::msg::ClientFeedback>("feedback_channel", 10);
        }

        void MessageHandler::handle_message(const std::uint8_t *message, int length)
        {
            converter::JsonSerializer msg_converter;
            std::pair<std::string, Message> publisher_msg;
            // Try to deserialize
            try
            {
                publisher_msg = msg_converter.deserialize(message, length);
            }
            catch (const std::exception &e)
            {
                // Send feedback, that deserializing didn't work
                RCLCPP_ERROR(this->get_logger(), "Error decoding: %s", e.what());
                ros2_api_msgs::msg::ClientFeedback feedback;
                feedback.message = "Error when deserializing message: " + std::string(e.what());
                feedback.status_code = FeedbackCode::UNEXPECTED_MSG_STRUCTURE;
                publisher_feedback_.get()->publish(feedback);
                return;
            }

            // Try to find publisher
            std::string name = publisher_msg.first;
            Message message_to_publish = publisher_msg.second;
            auto it = publisher_map_.find(name);
            if (it != publisher_map_.end())
            {
                // Send message
                std::unique_ptr<IMessagePublisher> &publisher = it->second;
                publisher.get()->publish(message_to_publish);
            }
            else
            {
                // Send feedback, that publisher wasn't found
                RCLCPP_ERROR(this->get_logger(), "Publisher with name '%s' not found.", name.c_str());
                ros2_api_msgs::msg::ClientFeedback feedback;
                feedback.message = "Publisher with name '" + name + "' not found.";
                feedback.status_code = FeedbackCode::PUBLISHER_NOT_FOUND;
                publisher_feedback_.get()->publish(feedback);
            }
        }
    } // namespace core
} // namespace ros2_api
