#ifndef TCP_SOCKET_HPP
#define TCP_SOCKET_HPP

#include <yaml-cpp/yaml.h>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <thread>
#include <atomic>
#include <string>
#include <cstdint>

namespace protocols
{
    /**
     * @class TCPSocket
     * @brief A class for handling TCP socket communication.
     *
     * This class provides methods to initialize, start, stop, and send messages
     * over a TCP socket. It is designed to be used as a plugin for the ROS2 framework.
     */
    class TCPSocket : public protocol_base::CommunicationProtocol
    {
    public:
        /**
         * @brief Destructor for TCPSocket.
         *
         * Cleans up resources and stops the socket if it is running.
         */
        ~TCPSocket();

        /**
         * @brief Sends a message to the connected client.
         *
         * @param message Pointer to the message data.
         * @param length Length of the message data.
         */
        void send_to_client(const std::uint8_t *message, int length) override;

        /**
         * @brief Starts the TCP socket.
         *
         * Initializes the socket and starts the thread for receiving data.
         */
        void start() override;

        /**
         * @brief Initializes the TCP socket with configuration parameters.
         *
         * @param config YAML node containing configuration parameters.
         */
        void initialize(const YAML::Node &config) override;

        /**
         * @brief Stops the TCP socket.
         *
         * Stops the receiving thread and closes the socket.
         */
        void stop() override;

    private:
        /**
         * @brief Handles communication with the connected client.
         *
         * Reads data from the client and invokes the callback function.
         */
        void handle_client();

        /**
         * @brief Starts the thread for receiving data from the socket.
         *
         * Creates and binds the socket, then listens for incoming connections.
         */
        void start_receiving();

        int socket_fd_;
        int client_fd_{-1};
        bool client_connected_{false};
        struct sockaddr_in server_addr_;
        std::string ip_address_;
        int port_;
        int queue_size_connections_;
        int max_message_size_;
        std::thread processing_thread_;
        std::atomic<bool> running_{false};
    };
} // namespace protocols

#endif // TCP_SOCKET_HPP
