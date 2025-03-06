#include <zmq.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api/protocols/zeromq.hpp>

// For pluginlib export.
#include <pluginlib/class_list_macros.hpp>

namespace protocols {

ZeroMQ::~ZeroMQ()
{
  stop();
}

void ZeroMQ::initialize(const YAML::Node &config)
{  
  context_ = zmq::context_t(1);
  socket_ = zmq::socket_t(context_, zmq::socket_type::pair);  
  endpoint_ = "tcp://127.0.0.1:5555";
  
  if (config && config["endpoint"]) {
    try {
      std::string ep = config["endpoint"].as<std::string>();
      if (!ep.empty()) {
        endpoint_ = ep;
      }
    }
    catch (const YAML::TypedBadConversion<std::string> &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error when setting endpoint.");
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ZeroMQ"),
                "No endpoint found in config. Using default: %s", endpoint_.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("ZeroMQ"),
              "ZeroMQ configured to bind at endpoint: %s", endpoint_.c_str());
}

void ZeroMQ::start()
{
  running_ = true;
  if (!processing_thread_.joinable()) {
    processing_thread_ = std::thread(&ZeroMQ::start_receiving, this);
  }
}

void ZeroMQ::stop()
{
  running_ = false;
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  try {
    socket_.close();
  } catch (const zmq::error_t &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error closing socket: %s", e.what());
  }
}

void ZeroMQ::send_to_client(const std::uint8_t *message, int length)
{
  if (!running_) {
    RCLCPP_WARN(rclcpp::get_logger("ZeroMQ"), "Cannot send message: socket not running.");
    return;
  }
  zmq::message_t msg(length);
  memcpy(msg.data(), message, length);
  try {
    socket_.send(msg, zmq::send_flags::none);
  } catch (const zmq::error_t &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error sending message: %s", e.what());
  }
}

void ZeroMQ::start_receiving()
{
  try {
    socket_.bind(endpoint_);
    RCLCPP_INFO(rclcpp::get_logger("ZeroMQ"), "ZeroMQ socket bound to %s", endpoint_.c_str());
  } catch (const zmq::error_t &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error binding socket: %s", e.what());
    return;
  }

  while (running_) {
    zmq::message_t msg;
    try {
      socket_.recv(msg, zmq::recv_flags::none);
      if (callback_) {
        callback_(static_cast<const std::uint8_t*>(msg.data()), msg.size());
      }
    } catch (const zmq::error_t &e) {
      RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error receiving message: %s", e.what());
    }
  }
}

} // namespace protocols

PLUGINLIB_EXPORT_CLASS(protocols::ZeroMQ, protocol_base::CommunicationProtocol)
