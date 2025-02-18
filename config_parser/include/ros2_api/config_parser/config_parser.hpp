#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <ros2_api/types/types.hpp>

namespace ros2_api {
namespace config {

/**
 * @brief Singleton class for parsing and validating the ROS2 API Config file
 */
class ConfigParser {
public:
    /**
     * @brief Get the singleton instance of ConfigParser
     * 
     * @return ConfigParser& Reference to the singleton instance
     */
    static ConfigParser& instance();
    
    /**
     * @brief Load the configuration from a path to a YAML file
     * 
     * @param config_path Path to the YAML configuration file
     * @return true if the configuration is successfully loaded and validated
     * @return false if there is an error loading or validating the configuration
     */
    bool load(const std::string& config_path);

    /**
     * @brief Get the transport plugin type
     * 
     * @return std::string Transport plugin type
     */
    std::string getTransportType() const;

    /**
     * @brief Get the state topic
     * 
     * @return std::string State topic
     */
    std::string getStateTopic() const;

    /**
     * @brief Get the transport parameters
     * 
     * @return YAML::Node Transport parameters
     */
    YAML::Node getTransportParams() const;

    /**
     * @brief Get the joint names
     * 
     * @return std::vector<std::string> Vector of joint names
     * @throws std::runtime_error if no joint names are set
     */
    std::vector<std::string> getJointNames() const;

    /**
     * @brief Get the base frame
     * 
     * @return std::string Base frame
     */
    std::string getBaseFrame() const;

    /**
     * @brief Get the publisher configuration
     * 
     * @return std::vector<MessageConfig> Vector of publisher configurations
     */
    std::vector<MessageConfig> getPublisherConfig() const;

    /**
     * @brief Check if calculated states are used
     * 
     * @return true if calculated states are used
     * @return false if calculated states are not used
     */
    bool useCalculatedStates() const;

    /**
     * @brief Reset the configuration to its default state
     */
    void reset();

private:
    /**
     * @brief Construct a new Config Parser object
     */
    ConfigParser();

    /**
     * @brief Destroy the Config Parser object
     */
    ~ConfigParser() = default;

    // Delete copy/move operations
    ConfigParser(const ConfigParser&) = delete;
    ConfigParser& operator=(const ConfigParser&) = delete;
    ConfigParser(ConfigParser&&) = delete;
    ConfigParser& operator=(ConfigParser&&) = delete;

    /**
     * @brief Validate the loaded configuration
     * 
     * @return true if the configuration is valid
     * @return false if the configuration is invalid
     */
    bool validate_config();

    /**
     * @brief Get the topic value from the YAML node
     * 
     * @param params YAML node containing the parameters
     * @param key Key to look for in the YAML node
     * @param default_value Default value to return if the key is not found
     * @return std::string Topic value
     */
    std::string getTopicValue(const YAML::Node& params, 
                            const std::string& key,
                            const std::string& default_value);

    /**
     * @brief Get the message type from a string
     * 
     * @param message_type String representation of the message type
     * @return MessageType Enum representation of the message type
     */
    MessageType getMessageType(std::string message_type);

    /**
     * @brief Get the default publisher topic
     * 
     * @param type Message type
     * @param name Name of the publisher
     * @return std::string Default publisher topic
     */
    std::string getDefaultPublisherTopic(MessageType type, std::string name);

    /**
     * @brief Replace a substring in a string with another string
     * 
     * @param str String to perform the replacement on
     * @param from Substring to replace
     * @param to Substring to replace with
     * @return true if the replacement was successful
     * @return false if the substring to replace was not found
     */
    bool replace(std::string& str, const std::string& from, const std::string& to);

    // Data members
    mutable std::mutex mutex_;
    YAML::Node root_node_;
    std::string config_path_;
    rclcpp::Logger logger_;
    std::string base_frame_;
    std::string state_topic_;
    std::string transport_plugin_name_;
    YAML::Node transport_params_;
    bool use_calculated_states_;
    std::vector<std::string> joint_names_;
    std::vector<MessageConfig> publisherConfig_;
};
} // namespace config
} // namespace ros2_api
#endif // CONFIG_PARSER_HPP