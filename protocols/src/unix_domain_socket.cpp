#include <ros2_api/protocols/unix_domain_socket.hpp>
#include <rclcpp/rclcpp.hpp>


namespace protocols {

  void UnixDomainSocket::initialize(const YAML::Node &config)
  {
    YAML::Node new_config;
    std::string socket_path = "/tmp/ros2_api.socket";  // Default socket

    try
    {
        std::string path = config["socket_path"].as<std::string>();
        if (!path.empty())
        {
            socket_path = path;
        }
    }
    catch (YAML::TypedBadConversion<std::string> &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error when setting socket.");
    }

    new_config["endpoint"] = "ipc://" + socket_path;
    ZeroMQ::initialize(new_config); 
  }
} // namespace protocols
PLUGINLIB_EXPORT_CLASS(protocols::UnixDomainSocket, protocol_base::CommunicationProtocol)