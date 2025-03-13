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

  // Get servo parameters
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  auto status_republisher_node = std::make_shared<robco_validator::ServoStatusRepublisher>(servo_parameters->status_topic);
  auto joy_republisher_node = std::make_shared<robco_validator::JoyRepublisher>(servo_parameters->cartesian_command_in_topic, servo_parameters->planning_frame);  
  auto jog_republisher_node = std::make_shared<robco_validator::JointJogRepublisher>(servo_parameters->joint_command_in_topic);
  node->set_parameter(rclcpp::Parameter("use_sim_time", servo_parameters->use_gazebo));
  status_republisher_node->set_parameter(rclcpp::Parameter("use_sim_time", servo_parameters->use_gazebo));
  joy_republisher_node->set_parameter(rclcpp::Parameter("use_sim_time", servo_parameters->use_gazebo));
  jog_republisher_node->set_parameter(rclcpp::Parameter("use_sim_time", servo_parameters->use_gazebo));


  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->providePlanningSceneService();
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  }

  auto servo = std::make_unique<moveit_servo::Servo>(node, servo_parameters, planning_scene_monitor);
  servo->start();


  robco_validator::CollisionObjectPublisher cop(node, servo_parameters->planning_frame);
  cop.publish_collision_objects();
  cop.publish_markers();


  
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->add_node(status_republisher_node);
  executor->add_node(joy_republisher_node);
  executor->add_node(jog_republisher_node);

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
