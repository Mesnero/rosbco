#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <robco_validator/collision_object_publisher.hpp>
#include <robco_validator/joy_republisher.hpp>
#include <robco_validator/servo_status_republisher.hpp>
#include <robco_validator/joint_jog_republisher.hpp>
#include <std_srvs/srv/trigger.hpp>

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


  //TODO: MAKE THIS WORK!
  robco_validator::CollisionObjectPublisher cop(node, servo_parameters->planning_frame);
  cop.publish_collision_objects();
  cop.publish_markers();

  // Start servo
  auto servo_start_client = node->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  servo_start_client->wait_for_service(std::chrono::seconds(1));
  servo_start_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->add_node(status_republisher_node);
  executor->add_node(joy_republisher_node);
  executor->add_node(jog_republisher_node);

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
