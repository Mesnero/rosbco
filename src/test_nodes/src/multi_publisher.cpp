#include "rclcpp/rclcpp.hpp"
#include "ros2_api_msgs/msg/calculated_states.hpp"  // Message from ros2_api_msgs package.
#include "geometry_msgs/msg/point.hpp"

#include <chrono>
#include <memory>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class MultiPublisherNode : public rclcpp::Node
{
public:
  MultiPublisherNode()
  : Node("multi_publisher_node")
  {
    // Create a publisher that will publish messages of type AllInterfaces.
    publisher_ = this->create_publisher<ros2_api_msgs::msg::CalculatedStates>("output", 10);

    // Create a timer to call the timer_callback every 2 seconds.
    timer_ = this->create_wall_timer(5s, std::bind(&MultiPublisherNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "MultiPublisherNode started.");
  }

private:
  void timer_callback()
  {
    // Create and populate the message.
    auto message = ros2_api_msgs::msg::CalculatedStates();

    // Fill in dummy data for demonstration purposes.
    message.name = {"joint1", "joint2", "joint3"};
    message.position_angle = {1.1, 2.2, 3.3};

    // Create some geometry_msgs::msg::Point objects.
    geometry_msgs::msg::Point point1;
    point1.x = 0.0;
    point1.y = 1.0;
    point1.z = 2.0;
    geometry_msgs::msg::Point point2;
    point2.x = 3.0;
    point2.y = 4.0;
    point2.z = 5.0;
    geometry_msgs::msg::Point point3;
    point3.x = 6.0;
    point3.y = 7.0;
    point3.z = 8.0;
    message.position_space = {point1, point2, point3};

    message.velocity = {0.1, 0.2, 0.3};
    message.acceleration = {0.01, 0.02, 0.03};
    message.jerk = {0.001, 0.002, 0.003};

    // Log and publish the message.
    publisher_->publish(message);
  }

  // Publisher and timer.
  rclcpp::Publisher<ros2_api_msgs::msg::CalculatedStates>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
