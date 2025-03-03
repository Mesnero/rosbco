#ifndef COLLISION_OBJECT_PUBLISHER_HPP
#define COLLISION_OBJECT_PUBLISHER_HPP
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

namespace robco_validator {

class CollisionObjectPublisher {
public:
    CollisionObjectPublisher() = delete;
    CollisionObjectPublisher(std::shared_ptr<rclcpp::Node> node, std::string planning_frame);
    ~CollisionObjectPublisher() = default;

    // Publish the collision objects to the planning scene
    void publish_collision_objects();
    // Publish the marker representation of the collision objects, since workspace cube would block vision in rviz
    void publish_markers();

private:
    struct Workspace {double min_x, min_y, min_z, max_x, max_y, max_z;};
    struct CollisionObj {double x, y , z, width, height, depth;  std::string name;};
    void declareAndGetParameters();
    moveit_msgs::msg::CollisionObject create_workspace();
    visualization_msgs::msg::Marker create_workspace_marker();
    moveit_msgs::msg::CollisionObject create_collision_object(CollisionObj col_obj);
    visualization_msgs::msg::Marker create_collision_object_marker(CollisionObj col_obj);


    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_objects_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<rclcpp::Node> node_;
    Workspace workspace_;
    std::vector<CollisionObj> collision_objs_;
    std::string planning_frame_;
    bool use_workspace_;
};

} // namespace robco_validator

#endif // COLLISION_OBJECT_PUBLISHER_HPP