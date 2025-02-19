#ifndef MESSAGE_PUBLISHER_HPP
#define MESSAGE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <ros2_api/publisher/message_publisher_interface.hpp>

/**
 * @file message_publisher.hpp
 * @brief Defines the TypedPublisher class template for publishing messages in ROS2.
 */

/**
 * @class TypedPublisher
 * @brief A template class for publishing messages of a specific type in ROS2.
 *
 * @tparam MsgType The type of the message to be published.
 */
template <typename MsgType>
class TypedPublisher : public IMessagePublisher
{
public:
  /**
   * @brief Constructs a new TypedPublisher object.
   *
   * @param node The shared pointer to the ROS2 node.
   * @param topic The name of the topic to publish messages to.
   * @param queue_size The size of the message queue.
   */
  TypedPublisher(const rclcpp::Node::SharedPtr &node,
                 const std::string &topic,
                 size_t queue_size)
  {
    publisher_ = node->create_publisher<MsgType>(topic, queue_size);
  }

  /**
   * @brief Publishes a message.
   *
   * @param msg A pointer to the message to be published.
   */
  void publish(const void *msg) override
  {
    publisher_->publish(*static_cast<const MsgType *>(msg));
  }

private:
  typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
};

#endif // MESSAGE_PUBLISHER_HPP