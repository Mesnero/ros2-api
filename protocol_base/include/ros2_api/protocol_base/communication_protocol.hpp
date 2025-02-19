#ifndef COMMUNICATION_PROTOCOL_HPP
#define COMMUNICATION_PROTOCOL_HPP

#include <functional>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>

/**
 * @file communication_protocol.hpp
 * @brief This file defines the CommunicationProtocol class, which serves as the base class for communication protocols in the ROS2 pluginlib framework.
 */

namespace protocol_base
{
    /**
     * @typedef CallbackType
     * @brief A type alias for the callback function used to handle incoming messages.
     *
     * The callback function takes a pointer to the message data and the length of the message.
     */
    using CallbackType = std::function<void(const std::uint8_t *message, int length)>;

    /**
     * @class CommunicationProtocol
     * @brief Abstract base class for communication protocols in the ROS2 pluginlib framework.
     *
     * This class defines the interface for communication protocols, including methods for initialization,
     * sending messages to clients, starting and stopping the protocol, and setting a callback for incoming messages.
     */
    class CommunicationProtocol
    {
    public:
        /**
         * @brief Virtual destructor for the CommunicationProtocol class.
         */
        virtual ~CommunicationProtocol() {}

        /**
         * @brief Initialize the communication protocol with the given configuration.
         *
         * @param config The YAML configuration node.
         */
        virtual void initialize(const YAML::Node &config) = 0;

        /**
         * @brief Send a message to the client.
         *
         * @param message Pointer to the message data.
         * @param length Length of the message.
         */
        virtual void send_to_client(const std::uint8_t *message, int length) = 0;

        /**
         * @brief Start the communication protocol.
         */
        virtual void start() = 0;

        /**
         * @brief Stop the communication protocol.
         */
        virtual void stop() = 0;

        /**
         * @brief Set the callback function to handle incoming messages.
         *
         * @param callback The callback function.
         */
        void set_callback(CallbackType callback)
        {
            callback_ = callback;
        }

    protected:
        CallbackType callback_; ///< The callback function for handling incoming messages.

        /**
         * @brief Protected constructor to prevent direct instantiation.
         */
        CommunicationProtocol() {}
    };

} // namespace protocol_base
#endif // COMMUNICATION_PROTOCOL_HPP