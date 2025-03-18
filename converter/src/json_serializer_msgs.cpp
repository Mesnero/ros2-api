#include <rclcpp/time.hpp>
#include <nlohmann/json.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>

#include <ros2_api/converter/json_serializer_msgs.hpp>
#include <ros2_api/config_parser/config_parser.hpp>


using json = nlohmann::json;
namespace ros2_api
{
    namespace converter
    {

        void to_json(json &j, const JointTrajectory w)
        {
            j["joint_traj_points"] = json::array();
            for (auto &traj_point : w.trajectory_msg.points)
            {
                json point =
                    json{
                        {"positions", traj_point.positions},
                        {"velocities", traj_point.velocities},
                        {"accelerations", traj_point.accelerations},
                        {"effort", traj_point.effort},
                        {"seconds", traj_point.time_from_start.sec},
                        {"nanoseconds", (int)traj_point.time_from_start.sec}};
                j["joint_traj_points"].push_back(point);
            }
        }

        void from_json(const json &j, JointTrajectory &w)
        {
            trajectory_msgs::msg::JointTrajectory traj_msg;
            std::vector<json> points;
            j.at("joint_traj_points").get_to(points);
            j.at("joint_names").get_to(traj_msg.joint_names);
            for (json traj_point : points)
            {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                traj_point.at("positions").get_to(point.positions);
                traj_point.at("velocities").get_to(point.velocities);
                traj_point.at("accelerations").get_to(point.accelerations);
                traj_point.at("effort").get_to(point.effort);
                builtin_interfaces::msg::Duration duration;
                traj_point.at("nanoseconds").get_to(duration.nanosec);
                traj_point.at("seconds").get_to(duration.sec);
                point.time_from_start = duration;
                traj_msg.points.push_back(point);
            }
            w.trajectory_msg = traj_msg;
        }

        void to_json(json &j, const JointGroupController g)
        {
            j["joint_values"] = g.positions.data;
        }

        void from_json(const json &j, JointGroupController &g)
        {
            j.at("joint_values").get_to(g.positions.data);
            g.positions.layout.dim.resize(1);
            g.positions.layout.dim[0].label = "data";
            g.positions.layout.dim[0].size = g.positions.data.size();
            g.positions.layout.dim[0].stride = g.positions.data.size();
            g.positions.layout.data_offset = 0;
        }

        void to_json(json &j, const Feedback f)
        {
            j["feedback_code"] = f.feedback.status_code;
            j["message"] = f.feedback.message;
        }

        void from_json(const json &j, Feedback &f)
        {
            j.at("feedback_code").get_to(f.feedback.status_code);
            j.at("message").get_to(f.feedback.message);
        }

        void to_json(json &j, const JointStates js)
        {
            j["nanoseconds"] = js.states.header.stamp.nanosec;
            j["seconds"] = js.states.header.stamp.sec;
            j["names"] = js.states.name;
            j["position"] = js.states.position;
            j["velocity"] = js.states.velocity;
            j["effort"] = js.states.effort;
        }

        void from_json(const json &j, JointStates &js)
        {
            j["nanoseconds"].get_to(js.states.header.stamp.nanosec);
            j["seconds"].get_to(js.states.header.stamp.sec);
            j.at("joint_names").get_to(js.states.name);
            j.at("position").get_to(js.states.position);
            j.at("effort").get_to(js.states.effort);
            j.at("velocity").get_to(js.states.velocity);
        }


        void to_json(json &j, const Joy joy) {
            std::vector<float> axesFloat = joy.joy.axes;
            std::vector<double> axesDouble(axesFloat.begin(), axesFloat.end());
            j["axes"] = axesDouble;
            j["buttons"] = joy.joy.buttons;
        } 

        void from_json(const json &j, Joy &joy) {
            std::vector<double> axesDouble;
            j.at("axes").get_to(axesDouble);
            joy.joy.axes.resize(axesDouble.size());
            std::transform(axesDouble.begin(), axesDouble.end(), joy.joy.axes.begin(), [](double val) {
                return static_cast<float>(val);
            });
            j.at("buttons").get_to(joy.joy.buttons);
        }

    } // namespace converter
} // namespace ros2_api