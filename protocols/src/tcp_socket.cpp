#include <ros2_api/protocols/tcp_socket.hpp>
#include <rclcpp/rclcpp.hpp>


namespace protocols {

  void TCPSocket::initialize(const YAML::Node &config)
  {
    YAML::Node new_config;
    std::string ip = "127.0.0.1";  // Default IP
    int port_recv = 5555;  // Default port for receiving
    int port_send = 5556;  // Default port for sending
    try {
        if (config["ip"])
        {
          ip = config["ip"].as<std::string>();
        }
        if (config["port_recv"])
        {
          port_recv = config["port_recv"].as<int>();
        }
        if (config["port_send"])
        {
          port_send = config["port_send"].as<int>();
        }
    }
    catch (YAML::TypedBadConversion<std::string> &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TCPSocket"), "Error when setting ip or port.");
    }

    new_config["endpoint_recv"] = "tcp://" + ip + ":" + std::to_string(port_recv);
    new_config["endpoint_send"] = "tcp://" + ip + ":" + std::to_string(port_send);
    ZeroMQ::initialize(new_config); 
  }
} // namespace protocols
PLUGINLIB_EXPORT_CLASS(protocols::TCPSocket, protocol_base::CommunicationProtocol)
