#ifndef MESSAGE_PUBLISHER_INTERFACE_HPP
#define MESSAGE_PUBLISHER_INTERFACE_HPP
#include <string>

/**
 * @file message_publisher_interface.hpp
 * @brief Interface for message publishers in ROS2.
 */

/**
 * @class IMessagePublisher
 * @brief Abstract base class for message publishers.
 *
 * This interface defines the contract for publishing messages in ROS2.
 */
class IMessagePublisher {
public:
  /**
   * @brief Virtual destructor for IMessagePublisher.
   */
  virtual ~IMessagePublisher() = default;
  
  /**
   * @brief Publish a message.
   *
   * This method publishes a message using a void pointer for type erasure.
   *
   * @param msg Pointer to the message to be published.
   */
  virtual void publish(const void* msg) = 0;
};

#endif // MESSAGE_PUBLISHER_INTERFACE_HPP