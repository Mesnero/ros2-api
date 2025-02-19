#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>
#include <stdexcept>
#include <unistd.h>

#include <ros2_api/config_parser/config_parser.hpp>
#include <ros2_api/types/types.hpp>

using namespace ros2_api::config;

// A test fixture to help with temporary file creation and cleanup.
class ConfigParserTest : public ::testing::Test
{
protected:
    // Path to the temporary file used in a test.
    std::string temp_file_;

    // Called before each test.
    void SetUp() override
    {
        // Nothing special to do here.
    }

    // Called after each test.
    void TearDown() override
    {
        if (!temp_file_.empty())
        {
            std::remove(temp_file_.c_str());
            temp_file_.clear();
        }
        // Reset the singleton ConfigParser::instance between tests.
        ConfigParser::instance().reset();
    }

    // Helper: Creates a temporary file with the given content and returns its file path.
    std::string createTempFile(const std::string &content)
    {
        // Template filename. mkstemps requires a mutable char array.
        char filename[] = "/tmp/test_config_XXXXXX.yaml";
        // The suffix length is 5 (".yaml")
        int fd = mkstemps(filename, 5);
        if (fd == -1)
        {
            throw std::runtime_error("Failed to create temporary file.");
        }
        // It is safe to close the file descriptor now; we'll use std::ofstream.
        close(fd);
        std::ofstream ofs(filename);
        ofs << content;
        ofs.close();
        temp_file_ = filename;
        return temp_file_;
    }
};

// Test a valid configuration file.
TEST_F(ConfigParserTest, ValidConfig)
{
    std::string yaml_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/joint_states"
    use_calculated_states: true
    joint_names:
      - "panda_joint1"
      - "panda_joint2"
    base_frame: "world"
    publishers:
      - publisher: "JointTrajectoryController"
        name: "controller1"
        topic: "/cmd_joint_trajectory"
    transport:
      type: "some_transport"
      params:
        param1: "value1"
        param2: 2
)";
    std::string file_path = createTempFile(yaml_content);
    EXPECT_TRUE(ConfigParser::instance().load(file_path));

    // Verify states topic (leading '/' stripped)
    EXPECT_EQ(ConfigParser::instance().get_state_topic(), "joint_states");
    // Verify the flag is set correctly.
    EXPECT_TRUE(ConfigParser::instance().use_calculated_states());

    // Verify joint names.
    std::vector<std::string> joint_names = ConfigParser::instance().get_joint_names();
    ASSERT_EQ(joint_names.size(), 2);
    EXPECT_EQ(joint_names[0], "panda_joint1");
    EXPECT_EQ(joint_names[1], "panda_joint2");

    // Verify base frame.
    EXPECT_EQ(ConfigParser::instance().get_base_frame(), "world");

    // Verify publisher configuration.
    std::vector<ros2_api::MessageConfig> pub_configs = ConfigParser::instance().get_publisher_config();
    ASSERT_EQ(pub_configs.size(), 1);
    EXPECT_EQ(pub_configs[0].name, "controller1");
    // (The MessageType will be set by the MessageTypeMapper and assumed valid.)

    // Verify transport settings.
    EXPECT_EQ(ConfigParser::instance().get_transport_type(), "some_transport");
    YAML::Node transport_params = ConfigParser::instance().get_transport_params();
    EXPECT_TRUE(transport_params["param1"]);
    EXPECT_TRUE(transport_params["param2"]);
    EXPECT_EQ(transport_params["param1"].as<std::string>(), "value1");
    EXPECT_EQ(transport_params["param2"].as<int>(), 2);
}

// Test when the YAML file is missing the top-level "ros2_api" key.
TEST_F(ConfigParserTest, MissingRos2ApiKey)
{
    std::string yaml_content = R"(
invalid_root:
  ros__parameters:
    states_topic: "/joint_states"
)";
    std::string file_path = createTempFile(yaml_content);
    EXPECT_FALSE(ConfigParser::instance().load(file_path));
}

// Test when the required "publishers" section is missing.
TEST_F(ConfigParserTest, MissingPublishers)
{
    std::string yaml_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/joint_states"
    joint_names:
      - "panda_joint1"
      - "panda_joint2"
    base_frame: "world"
    transport:
      type: "some_transport"
)";
    std::string file_path = createTempFile(yaml_content);
    EXPECT_FALSE(ConfigParser::instance().load(file_path));
}

// Test a publisher whose topic is invalid (does not begin with a '/').
TEST_F(ConfigParserTest, InvalidPublisherTopic)
{
    std::string yaml_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/joint_states"
    joint_names:
      - "panda_joint1"
      - "panda_joint2"
    base_frame: "world"
    publishers:
      - publisher: "JointTrajectoryController"
        name: "controller1"
        topic: "cmd_joint_trajectory"  # Invalid: no leading '/'
    transport:
      type: "some_transport"
)";
    std::string file_path = createTempFile(yaml_content);
    EXPECT_TRUE(ConfigParser::instance().load(file_path));
    std::vector<ros2_api::MessageConfig> pub_configs = ConfigParser::instance().get_publisher_config();
    ASSERT_EQ(pub_configs.size(), 1);
    // The ConfigParser::instance should have replaced the invalid topic with a default topic.
    EXPECT_NE(pub_configs[0].topic, "cmd_joint_trajectory");
}

// Test that calling get_joint_names() throws a runtime_error when joint_names is missing.
TEST_F(ConfigParserTest, MissingJointNames)
{
    std::string yaml_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/joint_states"
    publishers:
      - publisher: "JointTrajectoryController"
        name: "controller1"
        topic: "/cmd_joint_trajectory"
    transport:
      type: "some_transport"
)";
    std::string file_path = createTempFile(yaml_content);
    EXPECT_TRUE(ConfigParser::instance().load(file_path));
    EXPECT_THROW(ConfigParser::instance().get_joint_names(), std::runtime_error);
}

// Test that base_frame defaults correctly (to "map") when not specified.
TEST_F(ConfigParserTest, DefaultBaseFrame)
{
    std::string yaml_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/joint_states"
    joint_names:
      - "panda_joint1"
    publishers:
      - publisher: "JointTrajectoryController"
        name: "controller1"
        topic: "/cmd_joint_trajectory"
    transport:
      type: "some_transport"
)";
    std::string file_path = createTempFile(yaml_content);
    EXPECT_TRUE(ConfigParser::instance().load(file_path));
    EXPECT_EQ(ConfigParser::instance().get_base_frame(), "map");
}

// Test that the ConfigParser::instance fails if the transport type is missing.
TEST_F(ConfigParserTest, MissingTransportType)
{
    std::string yaml_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/joint_states"
    joint_names:
      - "panda_joint1"
    publishers:
      - publisher: "JointTrajectoryController"
        name: "controller1"
        topic: "/cmd_joint_trajectory"
    transport:
      params: {}
)";
    std::string file_path = createTempFile(yaml_content);
    EXPECT_FALSE(ConfigParser::instance().load(file_path));
}

// Test that a file with invalid YAML syntax causes load() to fail.
TEST_F(ConfigParserTest, InvalidYamlSyntax)
{
    // The YAML here is missing closing quotes/brackets.
    std::string yaml_content = R"(
ros2_api:
  ros__parameters:
    states_topic: "/joint_states
    joint_names: ["panda_joint1", "panda_joint2"
    publishers:
      - publisher: "JointTrajectoryController"
        name: "controller1"
        topic: "/cmd_joint_trajectory"
    transport:
      type: "some_transport"
)";
    std::string file_path = createTempFile(yaml_content);
    EXPECT_FALSE(ConfigParser::instance().load(file_path));
}

// Main function for GTest.
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
