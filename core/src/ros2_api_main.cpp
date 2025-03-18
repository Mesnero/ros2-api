#include <string>
#include <sys/stat.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_api/core/feedback_sender.hpp>
#include <ros2_api/core/message_handler.hpp>
#include <ros2_api/core/state_sender.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros2_api/protocol_base/communication_protocol.hpp>

std::string parse_config_file_path(int argc, char **argv)
{
  std::string path;
  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "--config_file" && (i + 1 < argc))
    {
      path = argv[i + 1];
      break;
    }
  }
  return path;
}

bool file_exists(const std::string &path)
{
  struct stat buffer;
  return (stat(path.c_str(), &buffer) == 0);
}

/**
 * @file ros2_api_main.cpp
 * @brief Entry point for the ROS2 API application.
 *
 * This file contains the main function which initializes the ROS2 environment,
 * loads configuration, sets up communication protocols, and starts the ROS2 nodes.
 */

/**
 * @brief Main function for the ROS2 API application.
 *
 * This function initializes the ROS2 system, parses the configuration file path from
 * command-line arguments, and loads the configuration. It then sets up the communication
 * protocol based on the configuration, initializes the necessary nodes, and starts the
 * ROS2 executor to handle node execution.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the application.
 *
 * The main steps performed by this function are:
 * - Initialize the ROS2 system.
 * - Parse the configuration file path from command-line arguments.
 * - Check if the configuration file path is specified and exists.
 * - Load the configuration using the ConfigParser.
 * - Create and initialize the communication protocol plugin.
 * - Set up the message handler, feedback sender, and state sender nodes.
 * - Add the nodes to a multi-threaded executor.
 * - Start the communication protocol and spin the executor.
 * - Stop the communication protocol and shut down the ROS2 system.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::string config_path = parse_config_file_path(argc, argv);

  if (config_path.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("LoadYaml"), "No --config_file specified!");
    rclcpp::shutdown();
  }

  if (!file_exists(config_path))
  {
    RCLCPP_ERROR(rclcpp::get_logger("LoadYaml"), "Yaml file doesn't exist.");
    rclcpp::shutdown();
  }

  ros2_api::config::ConfigParser::instance().load(config_path);
  std::string protocol_type = ros2_api::config::ConfigParser::instance().get_transport_type();

  std::shared_ptr<protocol_base::CommunicationProtocol> protocol;
  pluginlib::ClassLoader<protocol_base::CommunicationProtocol> comm_loader("protocol_base", "protocol_base::CommunicationProtocol");
  try
  {
    protocol = comm_loader.createSharedInstance(protocol_type);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PluginCreator"), "Error creating plugin %s. Error was %s", protocol_type.c_str(), ex.what());
    rclcpp::shutdown();
  }
  auto protocol_params = ros2_api::config::ConfigParser::instance().get_transport_params();
  protocol->initialize(protocol_params);

  auto parameter_node = std::make_shared<rclcpp::Node>("parameter_node");
  bool use_sim_time;
  parameter_node->get_parameter("use_sim_time", use_sim_time);

  auto publisher_config = ros2_api::config::ConfigParser::instance().get_publisher_config();
  auto message_handler_node = std::make_shared<ros2_api::core::MessageHandler>(protocol);
  message_handler_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
  message_handler_node.get()->set_up_publishers(publisher_config);

  auto feedback_sender_node = std::make_shared<ros2_api::core::FeedbackSender>(protocol);
  feedback_sender_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

  feedback_sender_node.get()->set_up_subscription();
  auto state_sender_node = std::make_shared<ros2_api::core::StateSender>(protocol);
  state_sender_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

  state_sender_node.get()->set_up_subscription();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(message_handler_node);
  executor.add_node(state_sender_node);
  executor.add_node(feedback_sender_node);
  protocol->start();
  executor.spin();

  protocol->stop();
  rclcpp::shutdown();
  return 0;
}