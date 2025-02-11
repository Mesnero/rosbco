#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <sstream>

class MultiSubscriberNode : public rclcpp::Node
{
public:
  MultiSubscriberNode() : Node("multi_subscriber_node")
  {
    // Subscribe to three Float64MultiArray topics.
    sub1_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "effort/commands", 10,
      std::bind(&MultiSubscriberNode::effortCallback, this, std::placeholders::_1));

    sub2_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "position/commands", 10,
      std::bind(&MultiSubscriberNode::positionCallback, this, std::placeholders::_1));

    sub3_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "velocity/commands", 10,
      std::bind(&MultiSubscriberNode::velocityCallback, this, std::placeholders::_1));

    // Subscribe to a JointTrajectory topic.
    sub_joint_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory_controller/joint_trajectory", 10,
      std::bind(&MultiSubscriberNode::jointTrajectoryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MultiSubscriberNode started.");
  }

private:
  // Utility function to print the contents of a Float64MultiArray.
  void printMultiArray(const std_msgs::msg::Float64MultiArray::SharedPtr msg, const std::string & topic_name)
  {
    std::stringstream ss;
    ss << "Received from " << topic_name << ": [";
    for (size_t i = 0; i < msg->data.size(); ++i)
    {
      ss << msg->data[i];
      if (i < msg->data.size() - 1)
      {
        ss << ", ";
      }
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  // Callbacks for the three Float64MultiArray topics.
  void effortCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    printMultiArray(msg, "effort/commands");
  }

  void positionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    printMultiArray(msg, "position/commands");
  }

  void velocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    printMultiArray(msg, "velocity/commands");
  }

  // Callback for the JointTrajectory topic.
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    std::stringstream ss;
    ss << "Received JointTrajectory:\n";
    
    // Print joint names.
    ss << "  Joint Names: ";
    for (const auto & name : msg->joint_names)
    {
      ss << name << " ";
    }
    ss << "\n";

    // Print the number of trajectory points.
    ss << "  Number of Trajectory Points: " << msg->points.size() << "\n";

    // If available, print details from the first trajectory point.
    if (!msg->points.empty())
    {
      ss << "  First Trajectory Point:\n";

      if (!msg->points[0].positions.empty())
      {
        ss << "    Positions: ";
        for (const auto & pos : msg->points[0].positions)
        {
          ss << pos << " ";
        }
        ss << "\n";
      }
      if (!msg->points[0].velocities.empty())
      {
        ss << "    Velocities: ";
        for (const auto & vel : msg->points[0].velocities)
        {
          ss << vel << " ";
        }
        ss << "\n";
      }
      if (!msg->points[0].accelerations.empty())
      {
        ss << "    Accelerations: ";
        for (const auto & acc : msg->points[0].accelerations)
        {
          ss << acc << " ";
        }
        ss << "\n";
      }
      ss << "    Time from start: " << msg->points[0].time_from_start.sec
         << " sec, " << msg->points[0].time_from_start.nanosec << " nsec\n";
    }

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  // Subscription objects.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub2_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub3_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_joint_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
