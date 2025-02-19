#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <future>
#include <nlohmann/json.hpp>

#include "ros2_api/core/message_handler.hpp"
#include "ros2_api/protocol_base/communication_protocol.hpp"
#include "ros2_api/config_parser/config_parser.hpp"
#include "ros2_api/converter/json_serializer.hpp"
#include "ros2_api_msgs/msg/client_feedback.hpp"
#include "ros2_api/types/types.hpp"
#include "ros2_api/publisher/publisher_factory.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;
using namespace ros2_api;
using namespace ros2_api::core;

class DummyProtocol : public protocol_base::CommunicationProtocol {
public:
  // This method simulates an incoming message by calling the callback.
  void send_to_handler(const std::uint8_t* msg, int length) {
    if(callback_) {
      callback_(msg, length);
    }
  }

  void send_to_client(const std::uint8_t* data, int length) override {}
  void initialize(const YAML::Node &config) override {}
  void start() override {}
  void stop() override {}
};

std::vector<std::uint8_t> create_valid_msgpack(const std::string &pub_name) {
  json j;
  j["type"] = MessageType::CLIENT_FEEDBACK;
  j["publisher_name"] = pub_name;
  j["payload"] = json{
    {"feedback_code", 200},
    {"message", "Test"}
  };
  return json::to_msgpack(j);
}

TEST(MessageHandlerIntegrationTest, DeserializationFailureFeedback) {
  auto dummy_protocol = std::make_shared<DummyProtocol>();
  auto handler = std::make_shared<ros2_api::core::MessageHandler>(dummy_protocol);
  handler->set_up_publishers({});

  auto node = rclcpp::Node::make_shared("feedback_subscriber_node");
  std::promise<ros2_api_msgs::msg::ClientFeedback> prom;
  auto fut = prom.get_future();
  auto sub = node->create_subscription<ros2_api_msgs::msg::ClientFeedback>(
    "feedback_channel", 10,
    [&prom](const ros2_api_msgs::msg::ClientFeedback::SharedPtr msg) {
      prom.set_value(*msg);
    }
  );

  // Simulate an invalid (empty) message.
  dummy_protocol->send_to_handler(nullptr, 0);
  rclcpp::spin_some(node);
  auto status = fut.wait_for(2s);
  EXPECT_EQ(status, std::future_status::ready);
  auto feedback = fut.get();
  EXPECT_EQ(feedback.status_code, FeedbackCode::UNEXPECTED_MSG_STRUCTURE);
  EXPECT_NE(feedback.message.find("Error when deserializing message:"), std::string::npos);
}

TEST(MessageHandlerIntegrationTest, PublisherNotFoundFeedback) {
  auto dummy_protocol = std::make_shared<DummyProtocol>();
  auto handler = std::make_shared<ros2_api::core::MessageHandler>(dummy_protocol);
  handler->set_up_publishers({});

  auto node = rclcpp::Node::make_shared("feedback_subscriber_node");
  std::promise<ros2_api_msgs::msg::ClientFeedback> prom;
  auto fut = prom.get_future();
  auto sub = node->create_subscription<ros2_api_msgs::msg::ClientFeedback>(
    "feedback_channel", 10,
    [&prom](const ros2_api_msgs::msg::ClientFeedback::SharedPtr msg) {
      prom.set_value(*msg);
    }
  );

  auto msgpack_data = create_valid_msgpack("nonexistent");
  dummy_protocol->send_to_handler(msgpack_data.data(), msgpack_data.size());
  rclcpp::spin_some(node);
  auto status = fut.wait_for(2s);
  EXPECT_EQ(status, std::future_status::ready);
  auto feedback = fut.get();
  EXPECT_EQ(feedback.status_code, FeedbackCode::PUBLISHER_NOT_FOUND);
  EXPECT_EQ(feedback.message, "Publisher with name 'nonexistent' not found.");
}

TEST(MessageHandlerIntegrationTest, PublisherExistsMessageSent) {
  MessageConfig cfg;
  cfg.msg_type = MessageType::CLIENT_FEEDBACK;
  cfg.name = "test_pub";
  cfg.topic = "/test_topic";

  auto dummy_protocol = std::make_shared<DummyProtocol>();
  auto handler = std::make_shared<ros2_api::core::MessageHandler>(dummy_protocol);
  handler->set_up_publishers({cfg});

  auto node = rclcpp::Node::make_shared("test_topic_subscriber_node");
  std::promise<ros2_api_msgs::msg::ClientFeedback> prom;
  auto fut = prom.get_future();
  auto sub = node->create_subscription<ros2_api_msgs::msg::ClientFeedback>(
    "/test_topic", 10,
    [&prom](const ros2_api_msgs::msg::ClientFeedback::SharedPtr msg) {
      prom.set_value(*msg);
    }
  );

  auto msgpack_data = create_valid_msgpack("test_pub");
  dummy_protocol->send_to_handler(msgpack_data.data(), msgpack_data.size());

  rclcpp::spin_some(node);
  auto status = fut.wait_for(2s);
  EXPECT_EQ(status, std::future_status::ready);
  auto received = fut.get();
  EXPECT_EQ(received.status_code, 200);
  EXPECT_EQ(received.message, "Test");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
