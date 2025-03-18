#include <ros2_api/protocols/unix_domain_socket.hpp>
#include <rclcpp/rclcpp.hpp>


namespace protocols {

  void UnixDomainSocket::initialize(const YAML::Node &config)
  {
    YAML::Node new_config;
    std::string socket_path_recv = "/tmp/ros2_api_recv.socket";  // Default socket recv
    std::string socket_path_send = "/tmp/ros2_api_send.socket";  // Default socket send

    try
    {
        std::string path_recv = config["socket_path_recv"].as<std::string>();
        if (!path_recv.empty())
        {
           socket_path_recv = path_recv;
        }
        std::string path_send = config["socket_path_send"].as<std::string>();
        if (!path_send.empty())
        {
           socket_path_send = path_send;
        }
    }
    catch (YAML::TypedBadConversion<std::string> &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error when setting socket.");
    }

    new_config["endpoint_recv"] = "ipc://" + socket_path_recv;
    new_config["endpoint_send"] = "ipc://" + socket_path_send;
    ZeroMQ::initialize(new_config); 
  }
} // namespace protocols
PLUGINLIB_EXPORT_CLASS(protocols::UnixDomainSocket, protocol_base::CommunicationProtocol)