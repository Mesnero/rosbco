#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <string>
#include <vector>
#include <array>

const std::string DEFAULT_PLANNING_FRAME = "base";
const double DEFAULT_MIN_X = -1.0;
const double DEFAULT_MAX_X = 1.0;
const double DEFAULT_MIN_Y = -1.0;
const double DEFAULT_MAX_Y = 1.0;
const double DEFAULT_MIN_Z = 0.0;
const double DEFAULT_MAX_Z = 2.0;

class WorkspaceBoundaries : public rclcpp::Node
{
public:
  WorkspaceBoundaries() : Node("workspace_boundaries")
  {
    // Declare parameters with sensible defaults
    declare_parameter<std::string>("frame_id", DEFAULT_PLANNING_FRAME);
    declare_parameter<double>("bounds.min_x", DEFAULT_MIN_X);
    declare_parameter<double>("bounds.max_x", DEFAULT_MAX_X);
    declare_parameter<double>("bounds.min_y", DEFAULT_MIN_Y);
    declare_parameter<double>("bounds.max_y", DEFAULT_MAX_Y);
    declare_parameter<double>("bounds.min_z", DEFAULT_MIN_Z);
    declare_parameter<double>("bounds.max_z", DEFAULT_MAX_Z);

    planning_frame_ = get_parameter("frame_id").as_string();
    min_x_ = get_parameter("bounds.min_x").as_double();
    max_x_ = get_parameter("bounds.max_x").as_double();
    min_y_ = get_parameter("bounds.min_y").as_double();
    max_y_ = get_parameter("bounds.max_y").as_double();
    min_z_ = get_parameter("bounds.min_z").as_double();
    max_z_ = get_parameter("bounds.max_z").as_double();
    RCLCPP_INFO(get_logger(), "Workspace bounds: [%f, %f], [%f, %f], [%f, %f]", min_x_, max_x_, min_y_, max_y_, min_z_,
                max_z_);

    if (min_x_ >= max_x_ || min_y_ >= max_y_ || min_z_ >= max_z_)
    {
      RCLCPP_WARN(get_logger(), "Invalid workspace bounds detected. Please ensure min < max for all dimensions.");
    }

    // Publishers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("workspace_marker", 10);
    collision_object_pub_ = this->create_publisher<moveit_msgs::msg::CollisionObject>("collision_object", 10);

    // Create and publish workspace visualization and collision object
    workspace_marker_ = createWorkspaceMarker();

    publishMarker();
    publishCollisionObject();

    RCLCPP_INFO(get_logger(), "WorkspaceBoundaries node started.");
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  visualization_msgs::msg::Marker workspace_marker_;

  std::string planning_frame_;
  double min_x_, max_x_;
  double min_y_, max_y_;
  double min_z_, max_z_;

  // Create visualization marker
  visualization_msgs::msg::Marker createWorkspaceMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.ns = "workspace_bounds";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;  // line thickness
    marker.color.r = 0.0;
    marker.color.g = 1.0;  // green lines
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Define corners
    std::vector<std::array<double, 3>> corners = { { min_x_, min_y_, min_z_ }, { min_x_, min_y_, max_z_ },
                                                   { min_x_, max_y_, min_z_ }, { min_x_, max_y_, max_z_ },
                                                   { max_x_, min_y_, min_z_ }, { max_x_, min_y_, max_z_ },
                                                   { max_x_, max_y_, min_z_ }, { max_x_, max_y_, max_z_ } };

    // Add edges
    auto add_edge = [&](int c1, int c2) {
      geometry_msgs::msg::Point p1, p2;
      p1.x = corners[c1][0];
      p1.y = corners[c1][1];
      p1.z = corners[c1][2];
      p2.x = corners[c2][0];
      p2.y = corners[c2][1];
      p2.z = corners[c2][2];
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    };

    add_edge(0, 1);
    add_edge(0, 2);
    add_edge(0, 4);
    add_edge(1, 3);
    add_edge(1, 5);
    add_edge(2, 3);
    add_edge(2, 6);
    add_edge(3, 7);
    add_edge(4, 5);
    add_edge(4, 6);
    add_edge(5, 7);
    add_edge(6, 7);

    return marker;
  }

  // Publish visualization marker
  void publishMarker()
  {
    workspace_marker_.header.stamp = this->now();
    marker_pub_->publish(workspace_marker_);
  }

  // Publish collision object
  void publishCollisionObject()
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.header.stamp = this->now();
    collision_object.id = "workspace_boundaries";

    const double wall_thickness = 0.05;  // Thickness of each wall

    auto add_wall = [&](double x, double y, double z, double dx, double dy, double dz) {
      shape_msgs::msg::SolidPrimitive wall;
      wall.type = shape_msgs::msg::SolidPrimitive::BOX;
      wall.dimensions = { dx, dy, dz };

      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;

      collision_object.primitives.push_back(wall);
      collision_object.primitive_poses.push_back(pose);
    };

    double z_center = (min_z_ + max_z_) / 2.0;
    double height = max_z_ - min_z_;  

    add_wall((min_x_ + max_x_) / 2.0, min_y_ - wall_thickness / 2.0, z_center, max_x_ - min_x_, wall_thickness, height);                          
    add_wall((min_x_ + max_x_) / 2.0, max_y_ + wall_thickness / 2.0, z_center, max_x_ - min_x_, wall_thickness, height);
    add_wall(min_x_ - wall_thickness / 2.0, (min_y_ + max_y_) / 2.0, z_center, wall_thickness, max_y_ - min_y_, height);
    add_wall(max_x_ + wall_thickness / 2.0, (min_y_ + max_y_) / 2.0, z_center, wall_thickness, max_y_ - min_y_, height);
    add_wall((min_x_ + max_x_) / 2.0, (min_y_ + max_y_) / 2.0, min_z_ - wall_thickness / 2.0, max_x_ - min_x_, max_y_ - min_y_, wall_thickness);
    add_wall((min_x_ + max_x_) / 2.0, (min_y_ + max_y_) / 2.0, max_z_ + wall_thickness / 2.0, max_x_ - min_x_, max_y_ - min_y_, wall_thickness);

    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_object_pub_->publish(collision_object);
    RCLCPP_INFO_ONCE(get_logger(), "Workspace collision walls published.");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WorkspaceBoundaries>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
