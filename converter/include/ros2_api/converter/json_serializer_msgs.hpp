#ifndef JSON_CONVERTERS_HPP
#define JSON_CONVERTERS_HPP

#include <rclcpp/time.hpp>
#include <nlohmann/json.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <ros2_api_msgs/msg/client_feedback.hpp>

#include <ros2_api/config_parser/config_parser.hpp>

using json = nlohmann::json;

namespace ros2_api
{
    namespace converter
    {
        /**
         * @brief This namespace contains functions and structures for converting ROS2 messages to and from JSON format.
         */
        
        /**
         * @brief Structure to hold JointTrajectory message.
         */
        struct JointTrajectory
        {
            trajectory_msgs::msg::JointTrajectory trajectory_msg;
        };

        /**
         * @brief Convert JointTrajectory to JSON.
         * @param j JSON object to store the converted data.
         * @param w JointTrajectory object to be converted.
         */
        void to_json(json &j, const JointTrajectory w);

        /**
         * @brief Convert JSON to JointTrajectory.
         * @param j JSON object containing the data.
         * @param w JointTrajectory object to store the converted data.
         */
        void from_json(const json &j, JointTrajectory &w);

        /**
         * @brief Structure to hold JointGroupController message.
         */
        struct JointGroupController
        {
            std_msgs::msg::Float64MultiArray positions;
        };

        /**
         * @brief Convert JointGroupController to JSON.
         * @param j JSON object to store the converted data.
         * @param g JointGroupController object to be converted.
         */
        void to_json(json &j, const JointGroupController g);

        /**
         * @brief Convert JSON to JointGroupController.
         * @param j JSON object containing the data.
         * @param g JointGroupController object to store the converted data.
         */
        void from_json(const json &j, JointGroupController &g);

        /**
         * @brief Structure to hold JointStates message.
         */
        struct JointStates
        {
            sensor_msgs::msg::JointState states;
        };

        /**
         * @brief Convert JointStates to JSON.
         * @param j JSON object to store the converted data.
         * @param js JointStates object to be converted.
         */
        void to_json(json &j, const JointStates js);

        /**
         * @brief Convert JSON to JointStates.
         * @param j JSON object containing the data.
         * @param js JointStates object to store the converted data.
         */
        void from_json(const json &j, JointStates &js);

        /**
         * @brief Structure to hold ClientFeedback message.
         */
        struct Feedback
        {
            ros2_api_msgs::msg::ClientFeedback feedback;
        };

        /**
         * @brief Convert Feedback to JSON.
         * @param j JSON object to store the converted data.
         * @param f Feedback object to be converted.
         */
        void to_json(json &j, const Feedback f);

        /**
         * @brief Convert JSON to Feedback.
         * @param j JSON object containing the data.
         * @param f Feedback object to store the converted data.
         */
        void from_json(const json &j, Feedback &f);

        /**
         * @brief Structure to hold a Joy message.
         */
        struct Joy
        {
            sensor_msgs::msg::Joy joy;
        };

        /**
         * @brief Convert Feedback to JSON.
         * @param j JSON object to store the converted data.
         * @param joy Joy object to be converted.
         */
        void to_json(json &j, const Joy joy);

        /**
         * @brief Convert JSON to Feedback.
         * @param j JSON object containing the data.
         * @param joy Joy object to store the converted data.
         */
        void from_json(const json &j, Joy &joy);

    } // namespace converter
} // namespace ros2_api

#endif // JSON_CONVERTERS_HPP
