#include "rclcpp/rclcpp.hpp"
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

  }


    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
