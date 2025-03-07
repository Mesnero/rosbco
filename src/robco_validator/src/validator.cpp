#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <robco_validator/collision_object_publisher.hpp>
#include <robco_validator/joy_republisher.hpp>
#include <robco_validator/servo_status_republisher.hpp>
#include <robco_validator/joint_jog_republisher.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(false);
  auto node = std::make_shared<rclcpp::Node>("validator_node", node_options);

  // Create the planning_scene_monitor
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node, "robot_description", tf_buffer, "planning_scene_monitor");
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning scene not configured.");
    return EXIT_FAILURE;
  }

  // Get servo parameters
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  robco_validator::CollisionObjectPublisher cop(node, servo_parameters->planning_frame);
  cop.publish_collision_objects();
  cop.publish_markers();

  // Start servo
  auto servo = std::make_unique<moveit_servo::Servo>(node, servo_parameters, planning_scene_monitor);
  servo->start();

  auto status_republisher_node = std::make_shared<robco_validator::ServoStatusRepublisher>(servo_parameters->status_topic);

  node->declare_parameter<bool>("use_joy_republisher", false);
  bool use_joy_republisher = node->get_parameter("use_joy_republisher").as_bool();

  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->add_node(status_republisher_node);

  auto joy_republisher_node = std::make_shared<robco_validator::JoyRepublisher>(servo_parameters->cartesian_command_in_topic, servo_parameters->planning_frame);  
  executor->add_node(joy_republisher_node);

  auto jog_republisher_node = std::make_shared<robco_validator::JointJogRepublisher>(servo_parameters->joint_command_in_topic);
  executor->add_node(jog_republisher_node);


  executor->spin();

  rclcpp::shutdown();
  return 0;
}
