#include <ros2_api/protocols/unix_domain_socket.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <cerrno>
#include <cstring>
#include <vector>

namespace protocols
{
    UnixDomainSocket::~UnixDomainSocket()
    {
        stop();
    }

    void UnixDomainSocket::start()
    {
        running_ = true;
        if (!processing_thread_.joinable())
        {
            processing_thread_ = std::thread([this]()
                                             { start_receiving(); });
        }
    }

    void UnixDomainSocket::stop()
    {
        RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "Stopped socket.");
        running_ = false;
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }

        if (socket_fd_ != -1)
        {
            close(socket_fd_);
        }
        unlink(socket_path_.c_str());
    }

    void UnixDomainSocket::start_receiving()
    {
        // Try to create socket
        socket_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
        if (socket_fd_ == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error creating socket");
            return;
        }

        // Setup socket
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "Creating socket at %s", socket_path_.c_str());
        strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
        unlink(socket_path_.c_str());

        // Bind socket to file path
        if (bind(socket_fd_, (sockaddr *)&addr, sizeof(addr)) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error binding socket");
            return;
        }
        if (listen(socket_fd_, queue_size_connections_) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error listening on socket");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "Ready for client to connect.");

        // Waiting for client connection
        while (running_)
        {
            client_fd_ = accept(socket_fd_, nullptr, nullptr);
            if (client_fd_ == -1)
            {
                continue;
            }
            client_connected_ = true;
            RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "Client connected");
            handle_client();
            close(client_fd_);
            client_fd_ = -1;
        }
    }

    void UnixDomainSocket::handle_client()
    {
        std::vector<std::uint8_t> buffer(max_message_size_);
        ssize_t bytes_read = 0;
        // Waiting for client to send message
        while (client_connected_)
        {
            memset(buffer.data(), 0, buffer.size());
            bytes_read = recv(client_fd_, buffer.data(), buffer.size(), 0);
            if (bytes_read == -1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error reading from client");
                break;
            }
            if (bytes_read == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "Client disconnected");
                client_connected_ = false;
                break;
            }
            callback_(buffer.data(), bytes_read);
        }
    }

    void UnixDomainSocket::send_to_client(const std::uint8_t *message, int length)
    {
        if (socket_fd_ != -1 && client_connected_)
        {
            if (write(client_fd_, message, length) == -1)
            {
                int err = errno;
                RCLCPP_ERROR(
                    rclcpp::get_logger("UnixDomainSocket"),
                    "Error writing to client: %s (errno: %d)",
                    std::strerror(err), err);
            }
        }
    }

    void UnixDomainSocket::initialize(const YAML::Node &config)
    {
        // Set default values
        socket_path_ = "/tmp/ros2_api.socket";
        queue_size_connections_ = 1;
        max_message_size_ = 1024;

        if (!config)
        {
            RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "No params given. Keeping defaults.");
        }

        // Socket path
        if (config["socket_path"])
        {
            try
            {
                std::string path = config["socket_path"].as<std::string>();
                if (!path.empty())
                {
                    socket_path_ = path;
                }
            }
            catch (YAML::TypedBadConversion<std::string> &ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error when setting socket.");
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "No socket_path found. Using default /tmp/ros2_api.socket");
        }

        // Queue size
        if (config["queue_size"])
        {
            try
            {
                queue_size_connections_ = config["queue_size"].as<int>();
            }
            catch (YAML::TypedBadConversion<int> &ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error when setting queue_size.");
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "No queue size found. Using default 1.");
        }

        // Max message size
        if (config["max_message_size"])
        {
            try
            {
                max_message_size_ = config["max_message_size"].as<int>();
            }
            catch (YAML::TypedBadConversion<int> &ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("UnixDomainSocket"), "Error when setting max_message_size.");
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("UnixDomainSocket"), "No max message size found. Using default 1024 bytes.");
        }
    }
} // namespace protocols

PLUGINLIB_EXPORT_CLASS(protocols::UnixDomainSocket, protocol_base::CommunicationProtocol)