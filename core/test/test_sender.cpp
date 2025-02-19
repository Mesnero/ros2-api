// test/test_feedback_and_state_sender.cpp

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <nlohmann/json.hpp>

#include "ros2_api/core/feedback_sender.hpp"
#include "ros2_api/core/state_sender.hpp"
#include "ros2_api/protocol_base/communication_protocol.hpp"
#include "ros2_api/config_parser/config_parser.hpp"
#include "ros2_api_msgs/msg/client_feedback.hpp"
#include "ros2_api_msgs/msg/calculated_states.hpp"
#include "ros2_api/converter/json_serializer.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;
using namespace ros2_api;
using namespace ros2_api::core;

// Dummy protocol that records the last sent data and simulates incoming messages.
class DummyProtocol : public protocol_base::CommunicationProtocol {
public:
  std::vector<std::uint8_t> last_sent_data;

  void send_to_client(const std::uint8_t* data, int length) override {
    last_sent_data.assign(data, data + length);
  }
  void initialize(const YAML::Node &config) override {}
  void start() override {}
  void stop() override {}
};

std::string create_temp_config(const std::string &content) {
  std::string filename = "/tmp/test_config.yaml";
  std::ofstream ofs(filename);
  ofs << content;
  ofs.close();
  return filename;
}

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

TEST(FeedbackAndStateSenderTest, FeedbackSenderSubscriptionCallback) {
  rclcpp::executors::SingleThreadedExecutor executor;

  auto dummy_protocol = std::make_shared<DummyProtocol>();

  auto feedback_sender = std::make_shared<ros2_api::core::FeedbackSender>(dummy_protocol);
  feedback_sender->set_up_subscription();
  executor.add_node(feedback_sender);

  auto pub_node = rclcpp::Node::make_shared("feedback_publisher_node");
  auto publisher = pub_node->create_publisher<ros2_api_msgs::msg::ClientFeedback>("feedback_channel", 10);
  executor.add_node(pub_node);

  ros2_api_msgs::msg::ClientFeedback fb;
  fb.status_code = 404;
  fb.message = "Feedback test message";
  publisher->publish(fb);

  // Spin until dummy_protocol receives data via send_to_client.
  auto start = std::chrono::steady_clock::now();
  while (dummy_protocol->last_sent_data.empty() && (std::chrono::steady_clock::now() - start < 2s)) {
      executor.spin_some(10ms);
  }
  EXPECT_FALSE(dummy_protocol->last_sent_data.empty());

  json j = json::from_msgpack(dummy_protocol->last_sent_data);
  EXPECT_EQ(j["type"], MessageType::CLIENT_FEEDBACK);
  if (j.contains("name"))
    EXPECT_EQ(j["name"].get<std::string>(), "feedback_channel");
  else if (j.contains("publisher_name"))
    EXPECT_EQ(j["publisher_name"].get<std::string>(), "feedback_channel");

  json payload = j["payload"];
  EXPECT_EQ(payload["feedback_code"].get<int>(), 404);
  EXPECT_EQ(payload["message"].get<std::string>(), "Feedback test message");
}

TEST(FeedbackAndStateSenderTest, StateSenderCalculatedStateSubscription) {
  std::string config_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/calc_state_topic"
    use_calculated_states: true
    publishers: 
      - publisher: "JointGroupPositionController"
        name: "test"
    transport:
      type: "dummy_transport"
      params: {}
)";
  auto config_file = create_temp_config(config_content);

  auto &config = ros2_api::config::ConfigParser::instance();
  config.reset();
  EXPECT_TRUE(config.load(config_file));

  rclcpp::executors::SingleThreadedExecutor executor;

  auto dummy_protocol = std::make_shared<DummyProtocol>();

  auto state_sender = std::make_shared<ros2_api::core::StateSender>(dummy_protocol);
  state_sender->set_up_subscription();
  executor.add_node(state_sender);

  auto pub_node = rclcpp::Node::make_shared("calc_state_publisher_node");
  auto publisher = pub_node->create_publisher<ros2_api_msgs::msg::CalculatedStates>("/calc_state_topic", 10);
  executor.add_node(pub_node);

  ros2_api_msgs::msg::CalculatedStates calc;
  calc.name = {"calc_state_test"};
  calc.acceleration = {1.0};
  calc.jerk = {1.0};
  calc.position_angle = {1.0};
  calc.position_space = std::vector<geometry_msgs::msg::Point>{};

  publisher->publish(calc);

  auto start = std::chrono::steady_clock::now();
  while (dummy_protocol->last_sent_data.empty() && (std::chrono::steady_clock::now() - start < 2s)) {
      executor.spin_some(10ms);
  }
  EXPECT_FALSE(dummy_protocol->last_sent_data.empty());

  json j = json::from_msgpack(dummy_protocol->last_sent_data);
  EXPECT_EQ(j["type"], MessageType::CALCULATED_STATES);
  EXPECT_EQ(j["name"].get<std::string>(), "calculated_states");

  json payload = j["payload"];
  ASSERT_TRUE(payload.contains("names"));
  EXPECT_EQ(payload["names"], json::array({"calc_state_test"}));
}

TEST(FeedbackAndStateSenderTest, StateSenderJointStateSubscription) {
  std::string config_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/joint_state_topic"
    use_calculated_states: false
    publishers: 
      - publisher: "JointGroupPositionController"
        name: "test"
    transport:
      type: "dummy_transport"
      params: {}
)";
  auto config_file = create_temp_config(config_content);

  auto &config = ros2_api::config::ConfigParser::instance();
  config.reset();
  EXPECT_TRUE(config.load(config_file));

  rclcpp::executors::SingleThreadedExecutor executor;

  auto dummy_protocol = std::make_shared<DummyProtocol>();

  auto state_sender = std::make_shared<ros2_api::core::StateSender>(dummy_protocol);
  state_sender->set_up_subscription();
  executor.add_node(state_sender);

  auto pub_node = rclcpp::Node::make_shared("joint_state_publisher_node");
  auto publisher = pub_node->create_publisher<sensor_msgs::msg::JointState>("/joint_state_topic", 10);
  executor.add_node(pub_node);

  sensor_msgs::msg::JointState joint;
  joint.name = {"joint_test"};
  joint.position = {1.23};

  publisher->publish(joint);

  auto start = std::chrono::steady_clock::now();
  while (dummy_protocol->last_sent_data.empty() && (std::chrono::steady_clock::now() - start < 2s)) {
      executor.spin_some(10ms);
  }
  EXPECT_FALSE(dummy_protocol->last_sent_data.empty());

  json j = json::from_msgpack(dummy_protocol->last_sent_data);
  EXPECT_EQ(j["type"], MessageType::JOINT_STATES);
  EXPECT_EQ(j["name"].get<std::string>(), "joint_states");

  json payload = j["payload"];
  ASSERT_TRUE(payload.contains("names"));
  ASSERT_TRUE(payload.contains("position"));
  EXPECT_EQ(payload["names"], json::array({"joint_test"}));
  EXPECT_EQ(payload["position"], json::array({1.23}));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
