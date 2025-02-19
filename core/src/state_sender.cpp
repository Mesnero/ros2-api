#include <ros2_api/core/state_sender.hpp>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api_msgs/msg/calculated_states.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <ros2_api/converter/json_serializer.hpp>
#include <string>

namespace ros2_api
{
    namespace core
    {
        StateSender::StateSender(std::shared_ptr<protocol_base::CommunicationProtocol> communication_protocol)
            : Node("state_sender"),
              communication_protocol_(std::move(communication_protocol))
        {
            state_topic_ = ros2_api::config::ConfigParser::instance().get_state_topic();
            use_calculated_states_ = ros2_api::config::ConfigParser::instance().use_calculated_states();
        }

        void StateSender::send_calculated_state(const ros2_api_msgs::msg::CalculatedStates &state)
        {
            converter::JsonSerializer msg_converter;
            std::vector<std::uint8_t> state_byte_aray = msg_converter.serialize(state);
            RCLCPP_INFO(this->get_logger(), "SENDING MESSAGE");
            communication_protocol_->send_to_client(state_byte_aray.data(), state_byte_aray.size());
        }

        void StateSender::send_joint_state(const sensor_msgs::msg::JointState &state)
        {
            converter::JsonSerializer msg_converter;
            std::vector<std::uint8_t> state_byte_array = msg_converter.serialize(state);
            communication_protocol_->send_to_client(state_byte_array.data(), state_byte_array.size());
        }

        void StateSender::set_up_subscription()
        {
            if (use_calculated_states_)
            {
                subscriber_calc_state_ = this->create_subscription<ros2_api_msgs::msg::CalculatedStates>(
                    state_topic_,
                    10,
                    [this](const ros2_api_msgs::msg::CalculatedStates::SharedPtr msg)
                    {
                        send_calculated_state(*msg);
                    });
            }
            else
            {
                subscriber_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    state_topic_,
                    10,
                    [this](const sensor_msgs::msg::JointState::SharedPtr msg)
                    {
                        send_joint_state(*msg);
                    });
            }
        }

    } // namespace core
} // namespace ros2