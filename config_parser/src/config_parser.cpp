#include <mutex>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include <ros2_api/types/types.hpp>
#include <ros2_api/config_parser/config_parser.hpp>

namespace ros2_api
{
    namespace config
    {

        ConfigParser::ConfigParser()
            : logger_(rclcpp::get_logger("ConfigParser"))
        {
        }

        ConfigParser &ConfigParser::instance()
        {
            static ConfigParser instance;
            return instance;
        }

        bool ConfigParser::load(const std::string &config_path)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            reset(); 
            try
            {
                root_node_ = YAML::LoadFile(config_path);
                config_path_ = config_path;
            }
            catch (const YAML::Exception &e)
            {
                RCLCPP_ERROR(logger_, "Error loading YAML file: %s", e.what());
                reset();

                return false;
            }
            return validate_config();
        }

        void ConfigParser::reset()
        {
            config_path_ = "";
            root_node_ = YAML::Node();
            publisher_config_.clear();
            transport_plugin_name_ = "";
            transport_params_ = YAML::Node();
            state_topic_ = "";
        }

        std::string ConfigParser::get_transport_type() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return transport_plugin_name_;
        }

        std::vector<MessageConfig> ConfigParser::get_publisher_config() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return publisher_config_;
        }

        std::string ConfigParser::ConfigParser::get_state_topic() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return state_topic_;
        }

        YAML::Node ConfigParser::get_transport_params() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return transport_params_;
        }

        bool ConfigParser::validate_config()
        {
            MessageTypeMapper mtm;

            // Check if the yaml begins with the right parameters
            if (!root_node_["ros2_api"] || !root_node_["ros2_api"]["ros__parameters"])
            {
                RCLCPP_ERROR(logger_, "Missing 'ros2_api' or 'ros__parameters' in YAML.");
                return false;
            }
            YAML::Node params = root_node_["ros2_api"]["ros__parameters"];

            // Get state topic value
            state_topic_ = get_topic_value(params, "states_topic", "joint_states");

            // Setup MessageConfig array for publishers
            if (!params["publishers"] || !params["publishers"].IsSequence())
            {
                RCLCPP_ERROR(logger_, "Invalid or missing publishers in YAML.");
                return false;
            }

            for (const auto &iface : params["publishers"])
            {
                MessageConfig cfg;
                if (!iface["publisher"])
                {
                    RCLCPP_ERROR(logger_, "No publisher specified. Skipping.");
                    continue;
                }
                std::string msgTypeStr = iface["publisher"].as<std::string>();
                MessageType type = mtm.getTypeFromString(msgTypeStr);
                if (type == MessageType::UNKNOWN)
                {
                    RCLCPP_ERROR(logger_, "Unknown publisher type. Skipping.");
                    continue;
                }
                cfg.msg_type = type;
                if (!iface["name"])
                {
                    RCLCPP_ERROR(logger_, "No name specified. Skipping.");
                    continue;
                }
                std::string name = iface["name"].as<std::string>();
                cfg.name = name;

                if (!iface["topic"])
                {
                    cfg.topic = mtm.getDefaultTopic(cfg.msg_type, cfg.name);
                    RCLCPP_INFO(logger_, "No topic specified. Using default '%s'.", cfg.topic.c_str());
                }
                else
                {
                    std::string topic = iface["topic"].as<std::string>();
                    if (topic[0] == '/')
                    {
                        cfg.topic = topic.substr(1);
                    }
                    else
                    {
                        cfg.topic = mtm.getDefaultTopic(cfg.msg_type, cfg.name);
                        RCLCPP_ERROR(logger_, "Invalid topic name: %s. Must start with /. Using default %s.", topic.c_str(), cfg.topic.c_str());
                    }
                }
                publisher_config_.push_back(cfg);
            }
            if (publisher_config_.empty())
            {
                RCLCPP_ERROR(logger_, "No publishers or only wrong ones specified.");
                return false;
            }

            // Parse config for protocol
            if (!params["transport"]["type"] || !params["transport"]["type"].IsScalar())
            {
                RCLCPP_ERROR(logger_, "Invalid or missing transport.type in YAML.");
                return false;
            }
            transport_plugin_name_ = params["transport"]["type"].as<std::string>();

            if (!params["transport"]["params"])
            {
                RCLCPP_INFO(logger_, "No params for transport plugin found.");
            }
            else
            {
                transport_params_ = params["transport"]["params"];
            }
            return true;
        }

        std::string ConfigParser::get_topic_value(const YAML::Node &params, const std::string &key, const std::string &default_value)
        {
            if (!params[key])
            {
                RCLCPP_INFO(logger_, "No states_topic found. Using default %s.", default_value.c_str());
                return default_value;
            }
            std::string topic = params[key].as<std::string>();
            if (topic[0] == '/')
            {
                return topic.substr(1);
            }
            RCLCPP_ERROR(logger_, "Invalid topic name: %s. Using default %s.", topic.c_str(), default_value.c_str());
            return default_value;
        }
    } // namespace config
} // namespace ros2_api
