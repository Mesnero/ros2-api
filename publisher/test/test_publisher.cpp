
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include "ros2_api/publisher/publisher_factory.hpp"
#include "ros2_api/types/types.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>

using namespace std::chrono_literals;
using ros2_api::MessageType;
using ros2_api::publisher::PublisherFactory;


// Test fixture for publisher factory tests.
class PublisherFactoryTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  void SetUp() override {
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
    node_ = rclcpp::Node::make_shared("publisher_factory_test_node");
    executor_.add_node(node_);
  }

  // Helper function to spin until a std::future is ready or a timeout occurs.
  template<typename T>
  bool wait_for_future(std::future<T>& fut, std::chrono::milliseconds timeout = 1000ms) {
    auto start = std::chrono::steady_clock::now();
    while (fut.wait_for(10ms) != std::future_status::ready) {
      executor_.spin_once(10ms);
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
    }
    return true;
  }
};

// Test for publisher type: JOINT_GROUP_POSITION_CONTROLLER.
TEST_F(PublisherFactoryTest, JointGroupPositionController) {
  std::string topic = "test_pg";
  auto publisher = PublisherFactory::create_publisher(node_, MessageType::JOINT_GROUP_POSITION_CONTROLLER, topic, 10);

  std::promise<std_msgs::msg::Float64MultiArray> prom;
  auto fut = prom.get_future();
  auto subscription = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic, 10,
    [&prom](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      prom.set_value(*msg);
    }
  );

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {1.23, 4.56};
  publisher->publish(&msg);

  bool received = wait_for_future(fut, 1000ms);
  ASSERT_TRUE(received);
  auto received_msg = fut.get();
  EXPECT_EQ(received_msg.data, msg.data);
}

// Test for publisher type: JOINT_GROUP_EFFORT_CONTROLLER.
TEST_F(PublisherFactoryTest, JointGroupEffortController) {
  std::string topic = "test_ge";
  auto publisher = PublisherFactory::create_publisher(node_, MessageType::JOINT_GROUP_EFFORT_CONTROLLER, topic, 10);

  std::promise<std_msgs::msg::Float64MultiArray> prom;
  auto fut = prom.get_future();
  auto subscription = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic, 10,
    [&prom](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      prom.set_value(*msg);
    }
  );

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {7.89, 0.12};
  publisher->publish(&msg);

  bool received = wait_for_future(fut, 1000ms);
  ASSERT_TRUE(received);
  auto received_msg = fut.get();
  EXPECT_EQ(received_msg.data, msg.data);
}

// Test for publisher type: JOINT_GROUP_VELOCITY_CONTROLLER.
TEST_F(PublisherFactoryTest, JointGroupVelocityController) {
  std::string topic = "test_gv";
  auto publisher = PublisherFactory::create_publisher(node_, MessageType::JOINT_GROUP_VELOCITY_CONTROLLER, topic, 10);

  std::promise<std_msgs::msg::Float64MultiArray> prom;
  auto fut = prom.get_future();
  auto subscription = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic, 10,
    [&prom](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      prom.set_value(*msg);
    }
  );

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {3.14, 2.71};
  publisher->publish(&msg);

  bool received = wait_for_future(fut, 1000ms);
  ASSERT_TRUE(received);
  auto received_msg = fut.get();
  EXPECT_EQ(received_msg.data, msg.data);
}

// Test for publisher type: JOINT_TRAJECTORY_CONTROLLER.
TEST_F(PublisherFactoryTest, JointTrajectoryController) {
  std::string topic = "test_jt";
  auto publisher = PublisherFactory::create_publisher(node_, MessageType::JOINT_TRAJECTORY_CONTROLLER, topic, 10);

  std::promise<trajectory_msgs::msg::JointTrajectory> prom;
  auto fut = prom.get_future();
  auto subscription = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    topic, 10,
    [&prom](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      prom.set_value(*msg);
    }
  );

  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.frame_id = "test_frame";
  publisher->publish(&msg);

  bool received = wait_for_future(fut, 1000ms);
  ASSERT_TRUE(received);
  auto received_msg = fut.get();
  EXPECT_EQ(received_msg.header.frame_id, msg.header.frame_id);
}

// Test for publisher type: CLIENT_FEEDBACK.
TEST_F(PublisherFactoryTest, ClientFeedback) {
  std::string topic = "test_cf";
  auto publisher = PublisherFactory::create_publisher(node_, MessageType::CLIENT_FEEDBACK, topic, 10);

  std::promise<ros2_api_msgs::msg::ClientFeedback> prom;
  auto fut = prom.get_future();
  auto subscription = node_->create_subscription<ros2_api_msgs::msg::ClientFeedback>(
    topic, 10,
    [&prom](const ros2_api_msgs::msg::ClientFeedback::SharedPtr msg) {
      prom.set_value(*msg);
    }
  );

  ros2_api_msgs::msg::ClientFeedback msg;
  msg.status_code = 200;
  msg.message = "Test feedback";
  publisher->publish(&msg);

  bool received = wait_for_future(fut, 1000ms);
  ASSERT_TRUE(received);
  auto received_msg = fut.get();
  EXPECT_EQ(received_msg.status_code, msg.status_code);
  EXPECT_EQ(received_msg.message, msg.message);
}

// Test that using an unknown MessageType causes the factory to throw.
TEST_F(PublisherFactoryTest, UnknownMessageTypeThrows) {
  std::string topic = "test_unknown";
  EXPECT_THROW({
    auto publisher = PublisherFactory::create_publisher(node_, static_cast<MessageType>(999), topic, 10);
  }, std::runtime_error);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
