#ifndef ZEROMQ_HPP
#define ZEROMQ_HPP

#include <yaml-cpp/yaml.h>
#include <ros2_api/protocol_base/communication_protocol.hpp>
#include <zmq.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

/**
 * @class ZeroMQ
 * @brief A class for handling ZeroMQ communication.
 *
 * This class provides methods to initialize, start, stop, and send messages
 * over ZeroMQ. It is designed to be used as a plugin for the ROS2 framework.
 */
namespace protocols
{
    class ZeroMQ : public protocol_base::CommunicationProtocol
    {
    public:
        /**
         * @brief Destructor for ZeroMQ.
         *
         * Cleans up resources and stops the socket if it is running.
         */
        ~ZeroMQ();

        /**
         * @brief Sends a message to the connected client.
         *
         * @param message Pointer to the message data.
         * @param length Length of the message data.
         */
        void send_to_client(const std::uint8_t *message, int length) override;

        /**
         * @brief Starts ZeroMQ.
         *
         * Initializes the socket and starts the thread for receiving data.
         */
        void start() override;

        /**
         * @brief Initializes the ZeroMQ with configuration parameters.
         *
         * @param config YAML node containing configuration parameters.
         */
        void initialize(const YAML::Node &config) override;

        /**
         * @brief Stops ZeroMQ.
         *
         * Stops the receiving thread and closes the socket.
         */
        void stop() override;
    
    protected:
        std::string endpoint_recv_;
        std::string endpoint_send_;

    private:

        /**
         * @brief Starts the thread for receiving data from the socket.
         *
         * Creates and binds the socket, then listens for incoming connections.
         */
        void start_receiving();

        std::atomic<bool> running_{false};
        std::thread processing_thread_;
        zmq::context_t context_;
        zmq::socket_t socket_recv_;
        zmq::socket_t socket_send_;

    };
} // namespace protocols
#endif // UNIX_DOMAIN_SOCKET_HPP