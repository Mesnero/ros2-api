#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <mutex>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include <ros2_api/types/types.hpp>

namespace ros2_api
{
    namespace config
    {

        /**
         * @brief Singleton class for parsing and validating the ROS2 API Config file
         */
        class ConfigParser
        {
        public:
            /**
             * @brief Get the singleton instance of ConfigParser
             *
             * @return ConfigParser& Reference to the singleton instance
             */
            static ConfigParser &instance();

            /**
             * @brief Load the configuration from a path to a YAML file
             *
             * @param config_path Path to the YAML configuration file
             * @return true if the configuration is successfully loaded and validated
             * @return false if there is an error loading or validating the configuration
             */
            bool load(const std::string &config_path);

            /**
             * @brief Get the transport plugin type
             *
             * @return std::string Transport plugin type
             */
            std::string get_transport_type() const;

            /**
             * @brief Get the state topic
             *
             * @return std::string State topic
             */
            std::string get_state_topic() const;

            /**
             * @brief Get the transport parameters
             *
             * @return YAML::Node Transport parameters
             */
            YAML::Node get_transport_params() const;


            /**
             * @brief Get the publisher configuration
             *
             * @return std::vector<MessageConfig> Vector of publisher configurations
             */
            std::vector<MessageConfig> get_publisher_config() const;

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
            ConfigParser(const ConfigParser &) = delete;
            ConfigParser &operator=(const ConfigParser &) = delete;
            ConfigParser(ConfigParser &&) = delete;
            ConfigParser &operator=(ConfigParser &&) = delete;

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
            std::string get_topic_value(const YAML::Node &params,
                                        const std::string &key,
                                        const std::string &default_value);

            // Data members
            mutable std::mutex mutex_;
            YAML::Node root_node_;
            std::string config_path_;
            rclcpp::Logger logger_;
            std::string state_topic_;
            std::string transport_plugin_name_;
            YAML::Node transport_params_;
            std::vector<MessageConfig> publisher_config_;
        };
    } // namespace config
} // namespace ros2_api
#endif // CONFIG_PARSER_HPP