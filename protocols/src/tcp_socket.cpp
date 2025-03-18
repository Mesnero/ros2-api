#include <ros2_api/protocols/tcp_socket.hpp>
#include <rclcpp/rclcpp.hpp>


namespace protocols {

  void TCPSocket::initialize(const YAML::Node &config)
  {
    YAML::Node new_config;
    std::string ip = "127.0.0.1";  // Default IP
    int port = 5555;  // Default port
    try {
        if (config["ip"])
        {
          ip = config["ip"].as<std::string>();
        }
        if (config["port"])
        {
          port = config["port"].as<int>();
        }
    }
    catch (YAML::TypedBadConversion<std::string> &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error when setting ip or port.");
    }

    new_config["endpoint"] = "tcp://" + ip + ":" + std::to_string(port);
    ZeroMQ::initialize(new_config); 
  }
} // namespace protocols
PLUGINLIB_EXPORT_CLASS(protocols::TCPSocket, protocol_base::CommunicationProtocol)
