#include <zmq.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <ros2_api/protocols/zeromq.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace protocols {

ZeroMQ::~ZeroMQ()
{
  stop();
}

void ZeroMQ::initialize(const YAML::Node &config)
{
  context_ = zmq::context_t(1);

  // Set default endpoints
  endpoint_recv_ = "tcp://127.0.0.1:5555";
  endpoint_send_ = "tcp://127.0.0.1:5556";

  if (config) {
    if (config["endpoint_recv"]) {
      try {
        std::string ep = config["endpoint_recv"].as<std::string>();
        if (!ep.empty()) {
          endpoint_recv_ = ep;
        }
      }
      catch (const YAML::TypedBadConversion<std::string> &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error when setting endpoint_recv.");
      }
    }
    if (config["endpoint_send"]) {
      try {
        std::string ep = config["endpoint_send"].as<std::string>();
        if (!ep.empty()) {
          endpoint_send_ = ep;
        }
      }
      catch (const YAML::TypedBadConversion<std::string> &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error when setting endpoint_send.");
      }
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ZeroMQ"),
              "ZeroMQ configured with endpoint_recv: %s, endpoint_send: %s",
              endpoint_recv_.c_str(), endpoint_send_.c_str());

  socket_recv_ = zmq::socket_t(context_, zmq::socket_type::pull);
  socket_send_ = zmq::socket_t(context_, zmq::socket_type::push);
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
    socket_recv_.close();
  } catch (const zmq::error_t &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error closing receiving socket: %s", e.what());
  }
  try {
    socket_send_.close();
  } catch (const zmq::error_t &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error closing sending socket: %s", e.what());
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
    RCLCPP_INFO(rclcpp::get_logger("ZeroMQ"), "Sending message to client");
    socket_send_.send(msg, zmq::send_flags::none);
  } catch (const zmq::error_t &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error sending message: %s", e.what());
  }
}

void ZeroMQ::start_receiving()
{
  try {
    socket_recv_.bind(endpoint_recv_);
    RCLCPP_INFO(rclcpp::get_logger("ZeroMQ"), "ZeroMQ receiving socket bound to %s", endpoint_recv_.c_str());
  } catch (const zmq::error_t &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error binding receiving socket: %s", e.what());
    return;
  }

  try {
    socket_send_.bind(endpoint_send_);
    RCLCPP_INFO(rclcpp::get_logger("ZeroMQ"), "ZeroMQ sending socket bound to %s", endpoint_send_.c_str());
  } catch (const zmq::error_t &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeroMQ"), "Error binding sending socket: %s", e.what());
    return;
  }

  while (running_) {
    zmq::message_t msg;
    try {
      socket_recv_.recv(msg, zmq::recv_flags::none);
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
