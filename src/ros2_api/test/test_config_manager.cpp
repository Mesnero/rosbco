#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <ros2_api/config_manager.hpp>
#include <rclcpp/rclcpp.hpp>

class ConfigManagerTest : public ::testing::Test 
{
protected:
    void TearDown() override 
    {
        // Reset singleton state between tests
        robot_api::ConfigManager::instance().reset();
    }

    bool loadFromString(const std::string& yaml_str) 
    {
        const std::string tmp_path = "/tmp/test_config.yaml";
        std::ofstream(tmp_path) << yaml_str;
        return robot_api::ConfigManager::instance().load(tmp_path);
    }

    robot_api::ConfigManager& manager = robot_api::ConfigManager::instance();
};

// --- Test Cases --- //

TEST_F(ConfigManagerTest, LoadValidConfig) 
{
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        joints_topic: "/custom_joints"
        command_interfaces:
          interfaces: ["velocity"]
          velocity_topic: "/custom_vel"
        transport:
          type: "test_plugin"
    )";

    EXPECT_TRUE(loadFromString(yaml));
    EXPECT_EQ(manager.getStateTopic(), "/custom_joints");
    EXPECT_EQ(manager.getTransportType(), "test_plugin");
}

TEST_F(ConfigManagerTest, HandlesMissingRosApiSection) {
    const std::string yaml = "other_key: value";
    EXPECT_FALSE(loadFromString(yaml));
    // Verify error logging
}

TEST_F(ConfigManagerTest, RejectsInvalidInterfaceType) {
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        command_interfaces:
          interfaces: ["invalid"]
        transport:
          type: "test_plugin"
    )";
    
    EXPECT_FALSE(loadFromString(yaml));
}

TEST_F(ConfigManagerTest, FixesInvalidTopicNames) {
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        joints_topic: "invalid_topic"
        command_interfaces:
          interfaces: ["velocity"]
          velocity_topic: "no_slash"
        transport:
          type: "test_plugin"
    )";
    
    loadFromString(yaml);
    EXPECT_EQ(manager.getStateTopic(), "/calc_joint_states"); // Default
    EXPECT_EQ(manager.getVelocityTopic(), "/command_velocity"); // Default
}

TEST_F(ConfigManagerTest, HandlesMissingTransportType) {
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        command_interfaces:
          interfaces: ["velocity"]
        transport: {}
    )";
    
    EXPECT_FALSE(loadFromString(yaml));
}

TEST_F(ConfigManagerTest, HandlesEmptyConfig) {
    EXPECT_FALSE(loadFromString(""));
}

TEST_F(ConfigManagerTest, HandlesMaxInterfaces) {
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        command_interfaces:
          interfaces: ["velocity", "position", "effort"]
        transport:
          type: "test_plugin"
    )";
    
    EXPECT_TRUE(loadFromString(yaml));
    EXPECT_EQ(manager.getCommandInterfaces().size(), 3);
}

TEST_F(ConfigManagerTest, ThreadSafety) {
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        joints_topic: "/custom_joints"
        command_interfaces:
          interfaces: ["velocity"]
          velocity_topic: "/custom_vel"
        transport:
          type: "test_plugin"
    )";
    loadFromString(yaml);

    std::vector<std::thread> threads;
    for(int i = 0; i < 10; ++i) {
        threads.emplace_back([&]() {
            EXPECT_NO_THROW(manager.getTransportType());
        });
    }
    
    for(auto& t : threads) t.join();
}

TEST_F(ConfigManagerTest, SetsPositionTopicDefault) {
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        command_interfaces:
          interfaces: ["position"]
        transport:
          type: "test_plugin"
    )";
    
    loadFromString(yaml);
    EXPECT_EQ(manager.getPositionTopic(), "/command_position");
}

TEST_F(ConfigManagerTest, RejectsNonSequenceInterfaces) {
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        command_interfaces:
          interfaces: "velocity"  # Should be list
        transport:
          type: "test_plugin"
    )";
    
    EXPECT_FALSE(loadFromString(yaml));
}

TEST_F(ConfigManagerTest, HandlesComplexTransportParams) {
    const std::string yaml = R"(
    ros2_api:
      ros__parameters:
        command_interfaces:
          interfaces: ["position"]
        transport:
          type: "complex_plugin"
          params:
            nested:
              key1: [1,2,3]
              key2: {a: b}
    )";
    
    loadFromString(yaml);
    auto params = manager.getTransportParams();
    EXPECT_TRUE(params["nested"]["key1"].IsSequence());
}

TEST_F(ConfigManagerTest, HandlesConfigNotSetUp) {
    auto emptyTopic = manager.getEffortTopic();
    ASSERT_EQ("", emptyTopic);
}

int main(int argc, char** argv) 
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}