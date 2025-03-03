#include <gtest/gtest.h>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <string>
#include <nlohmann/json.hpp>

#include <ros2_api/converter/json_serializer_msgs.hpp>
#include <ros2_api/converter/json_serializer.hpp>
#include <ros2_api/config_parser/config_parser.hpp>
#include <ros2_api/types/types.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ros2_api_msgs/msg/client_feedback.hpp>

using json = nlohmann::json;
using namespace ros2_api;
using namespace ros2_api::converter;
using namespace ros2_api::config;

// Helper function: load a temporary configuration file so that conversion
// functions (e.g., from_json for JointTrajectory and JointStates) can retrieve
// joint names and base frame from the singleton ConfigParser.
static bool setTestConfig() {
  std::string config_content = R"(
ros2_api:
  ros__parameters:
    joint_names: ["joint1", "joint2"]
    base_frame: "test_frame"
    publishers:
      - publisher: "JointGroupPositionController"
        name: "dummy"
        topic: "/dummy"
    transport:
      type: "dummy_transport"
      params: {}
)";
  std::string filename = "/tmp/test_config.yaml";
  std::ofstream ofs(filename);
  ofs << config_content;
  ofs.close();
  ConfigParser::instance().reset();
  return ConfigParser::instance().load(filename);
}


// Test 1: JointTrajectory to_json
TEST(JsonConversionTest, JointTrajectoryToJson) {
  JointTrajectory jt;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {1.0, 2.0};
  point.velocities = {0.1, 0.2};
  point.accelerations = {0.01, 0.02};
  point.effort = {0.5, 0.6};
  builtin_interfaces::msg::Duration dur;
  dur.sec = 3;
  dur.nanosec = 500;  
  point.time_from_start = dur;
  jt.trajectory_msg.points.push_back(point);

  json j;
  to_json(j, jt);
  ASSERT_TRUE(j.contains("joint_traj_points"));
  ASSERT_TRUE(j["joint_traj_points"].is_array());
  ASSERT_EQ(j["joint_traj_points"].size(), 1);
  auto jpoint = j["joint_traj_points"][0];
  EXPECT_EQ(jpoint["positions"], json::array({1.0, 2.0}));
  EXPECT_EQ(jpoint["velocities"], json::array({0.1, 0.2}));
  EXPECT_EQ(jpoint["accelerations"], json::array({0.01, 0.02}));
  EXPECT_EQ(jpoint["effort"], json::array({0.5, 0.6}));
  EXPECT_EQ(jpoint["seconds"], 3);
  EXPECT_EQ(jpoint["nanoseconds"], 3);
}

// Test 2: JointTrajectory from_json
TEST(JsonConversionTest, JointTrajectoryFromJson) {
  ASSERT_TRUE(setTestConfig());
  json j;
  j["joint_traj_points"] = json::array({
    {
      {"positions", {3.0, 4.0}},
      {"velocities", {0.3, 0.4}},
      {"accelerations", {0.03, 0.04}},
      {"effort", {0.7, 0.8}},
      {"seconds", 5},
      {"nanoseconds", 600}
    }
  });
  JointTrajectory jt;
  from_json(j, jt);
  ASSERT_EQ(jt.trajectory_msg.points.size(), 1);
  auto &pt = jt.trajectory_msg.points[0];
  EXPECT_EQ(pt.positions, std::vector<double>({3.0, 4.0}));
  EXPECT_EQ(pt.velocities, std::vector<double>({0.3, 0.4}));
  EXPECT_EQ(pt.accelerations, std::vector<double>({0.03, 0.04}));
  EXPECT_EQ(pt.effort, std::vector<double>({0.7, 0.8}));
  EXPECT_EQ(pt.time_from_start.sec, 5);
  EXPECT_EQ(pt.time_from_start.nanosec, 600);
  EXPECT_EQ(jt.trajectory_msg.header.frame_id, "test_frame");
  EXPECT_EQ(jt.trajectory_msg.joint_names, std::vector<std::string>({"joint1", "joint2"}));
}

// Test 3: JointGroupController to_json
TEST(JsonConversionTest, JointGroupControllerToJson) {
  JointGroupController jgc;
  jgc.positions.data = {1.1, 2.2, 3.3};
  json j;
  to_json(j, jgc);
  EXPECT_EQ(j["joint_values"], json::array({1.1, 2.2, 3.3}));
}

// Test 4: JointGroupController from_json
TEST(JsonConversionTest, JointGroupControllerFromJson) {
  json j = {
    {"joint_values", {4.4, 5.5, 6.6}}
  };
  JointGroupController jgc;
  from_json(j, jgc);
  EXPECT_EQ(jgc.positions.data, std::vector<double>({4.4, 5.5, 6.6}));
  ASSERT_EQ(jgc.positions.layout.dim.size(), 1);
  EXPECT_EQ(jgc.positions.layout.dim[0].label, "data");
  EXPECT_EQ(jgc.positions.layout.dim[0].size, 3);
  EXPECT_EQ(jgc.positions.layout.dim[0].stride, 3);
  EXPECT_EQ(jgc.positions.layout.data_offset, 0);
}

// Test 6: Feedback to_json
TEST(JsonConversionTest, FeedbackToJson) {
  Feedback fb;
  fb.feedback.status_code = 100;
  fb.feedback.message = "Test feedback";
  json j;
  to_json(j, fb);
  EXPECT_EQ(j["feedback_code"], 100);
  EXPECT_EQ(j["message"], "Test feedback");
}

// Test 7: Feedback from_json
TEST(JsonConversionTest, FeedbackFromJson) {
  json j = {
    {"feedback_code", 200},
    {"message", "All good"}
  };
  Feedback fb;
  from_json(j, fb);
  EXPECT_EQ(fb.feedback.status_code, 200);
  EXPECT_EQ(fb.feedback.message, "All good");
}

// Test 8: JointStates to_json
TEST(JsonConversionTest, JointStatesToJson) {
  JointStates js;
  js.states.name = {"jointA", "jointB"};
  js.states.position = {0.5, 1.5};
  js.states.velocity = {0.1, 0.2};
  js.states.effort = {5.0, 6.0};
  json j;
  to_json(j, js);
  EXPECT_EQ(j["names"], json::array({"jointA", "jointB"}));
  EXPECT_EQ(j["position"], json::array({0.5, 1.5}));
  EXPECT_EQ(j["velocity"], json::array({0.1, 0.2}));
  EXPECT_EQ(j["effort"], json::array({5.0, 6.0}));
}

// Test 9: JointStates from_json
TEST(JsonConversionTest, JointStatesFromJson) {
  ASSERT_TRUE(setTestConfig());
  json j = {
    {"position", {1.0, 2.0}},
    {"velocity", {0.3, 0.4}},
    {"effort", {10.0, 20.0}}
  };
  JointStates js;
  from_json(j, js);
  EXPECT_EQ(js.states.name, std::vector<std::string>({"joint1", "joint2"}));
  EXPECT_EQ(js.states.header.frame_id, "test_frame");
  EXPECT_EQ(js.states.position, std::vector<double>({1.0, 2.0}));
  EXPECT_EQ(js.states.velocity, std::vector<double>({0.3, 0.4}));
  EXPECT_EQ(js.states.effort, std::vector<double>({10.0, 20.0}));
}

// JsonSerializer tests

// Test 10: JsonSerializer::serialize for ClientFeedback
TEST(JsonSerializerTest, SerializeClientFeedback) {
  ros2_api_msgs::msg::ClientFeedback msg;
  msg.status_code = 404;
  msg.message = "Not Found";
  auto data = JsonSerializer::serialize(msg);
  json j = json::from_msgpack(data);
  EXPECT_EQ(j["type"], MessageType::CLIENT_FEEDBACK);
  EXPECT_EQ(j["name"], "feedback_channel");
  EXPECT_EQ(j["payload"]["feedback_code"], 404);
  EXPECT_EQ(j["payload"]["message"], "Not Found");
}

// Test 11: JsonSerializer::serialize for JointState
TEST(JsonSerializerTest, SerializeJointState) {
  sensor_msgs::msg::JointState msg;
  msg.name = {"joint1", "joint2"};
  msg.position = {1.0, 2.0};
  msg.velocity = {0.1, 0.2};
  msg.effort = {5.0, 6.0};
  auto data = JsonSerializer::serialize(msg);
  json j = json::from_msgpack(data);
  EXPECT_EQ(j["type"], MessageType::JOINT_STATES);
  EXPECT_EQ(j["name"], "joint_states");
  EXPECT_EQ(j["payload"]["names"], json::array({"joint1", "joint2"}));
  EXPECT_EQ(j["payload"]["position"], json::array({1.0, 2.0}));
  EXPECT_EQ(j["payload"]["velocity"], json::array({0.1, 0.2}));
  EXPECT_EQ(j["payload"]["effort"], json::array({5.0, 6.0}));
}

// Test 12: JsonSerializer::deserialize for a known type (ClientFeedback)
TEST(JsonSerializerTest, DeserializeClientFeedback) {
  json j = {
    {"type", MessageType::CLIENT_FEEDBACK},
    {"publisher_name", "test_feedback"},
    {"payload", {
        {"feedback_code", 123},
        {"message", "Test Message"}
    }}
  };
  auto msgpack = json::to_msgpack(j);
  auto result = JsonSerializer::deserialize(msgpack.data(), static_cast<int>(msgpack.size()));
  EXPECT_EQ(result.first, "test_feedback");
  Feedback* fb = static_cast<Feedback*>(const_cast<void*>(result.second));
  ASSERT_NE(fb, nullptr);
  EXPECT_EQ(fb->feedback.status_code, 123);
  EXPECT_EQ(fb->feedback.message, "Test Message");
  delete fb; 
}

// Main entry point for testing
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
