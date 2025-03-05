#include <robco_validator/collision_object_publisher.hpp>

namespace robco_validator {

CollisionObjectPublisher::CollisionObjectPublisher(std::shared_ptr<rclcpp::Node> node, std::string planning_frame)
: node_(node), planning_frame_(planning_frame) {
    declareAndGetParameters();
    collision_objects_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("object_markers", 10);
}

void CollisionObjectPublisher::declareAndGetParameters() {
    node_->declare_parameter<bool>("workspace.use_workspace", true);
    node_->declare_parameter<double>("workspace.min_x", -10.0); 
    node_->declare_parameter<double>("workspace.max_x", 10.0); 
    node_->declare_parameter<double>("workspace.min_y", -10.0); 
    node_->declare_parameter<double>("workspace.max_y", 10.0); 
    node_->declare_parameter<double>("workspace.min_z", -10.0); 
    node_->declare_parameter<double>("workspace.max_z", 10.0); 

    node_->get_parameter("workspace.use_workspace", use_workspace_);
    node_->get_parameter("workspace.min_x", workspace_.min_x);
    node_->get_parameter("workspace.max_x", workspace_.max_x);
    node_->get_parameter("workspace.min_y", workspace_.min_y);
    node_->get_parameter("workspace.max_y", workspace_.max_y);
    node_->get_parameter("workspace.min_z", workspace_.min_z);
    node_->get_parameter("workspace.max_z", workspace_.max_z);

    node_->declare_parameter<std::vector<std::string>>("collision_objects.names", std::vector<std::string>());
    std::vector<std::string> object_names;
    node_->get_parameter("collision_objects.names", object_names);
    for (const auto& name : object_names)
    {
      CollisionObj obj{};
      obj.name = name;
      std::string prefix = "collision_objects." + name;
      node_->declare_parameter<double>(prefix + ".x", 0.0);
      node_->declare_parameter<double>(prefix + ".y", 0.0);
      node_->declare_parameter<double>(prefix + ".z", 0.0);
      node_->declare_parameter<double>(prefix + ".depth", 1.0);
      node_->declare_parameter<double>(prefix + ".height", 1.0);
      node_->declare_parameter<double>(prefix + ".width", 1.0);
      node_->get_parameter(prefix + ".x", obj.x);
      node_->get_parameter(prefix + ".y", obj.y);
      node_->get_parameter(prefix + ".z", obj.z);
      node_->get_parameter(prefix + ".depth", obj.depth);
      node_->get_parameter(prefix + ".height", obj.height);
      node_->get_parameter(prefix + ".width", obj.width);
      collision_objs_.push_back(obj);
    }
}

moveit_msgs::msg::CollisionObject CollisionObjectPublisher::create_workspace() {
    moveit_msgs::msg::CollisionObject workspace_obj;
    workspace_obj.header.frame_id = planning_frame_;
    workspace_obj.header.stamp = node_->now();
    workspace_obj.id = "workspace";
    workspace_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    const double wall_thickness = 0.05;  // Thickness of each wall
    auto add_wall = [&](double x, double y, double z, double dx, double dy, double dz) {
      shape_msgs::msg::SolidPrimitive wall;
      wall.type = shape_msgs::msg::SolidPrimitive::BOX;
      wall.dimensions = { dx, dy, dz };

      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;

      workspace_obj.primitives.push_back(wall);
      workspace_obj.primitive_poses.push_back(pose);
    };

    double min_z = workspace_.min_z;
    double min_x = workspace_.min_x;
    double min_y = workspace_.min_y;
    double max_x = workspace_.max_x;
    double max_z = workspace_.max_z;
    double max_y = workspace_.max_y;


    double z_center = (min_z + max_z) / 2.0;
    double height = max_z - min_z;  

    add_wall((min_x + max_x) / 2.0, min_y - wall_thickness / 2.0, z_center, max_x - min_x, wall_thickness, height);                          
    add_wall((min_x + max_x) / 2.0, max_y + wall_thickness / 2.0, z_center, max_x - min_x, wall_thickness, height);
    add_wall(min_x - wall_thickness / 2.0, (min_y + max_y) / 2.0, z_center, wall_thickness, max_y - min_y, height);
    add_wall(max_x + wall_thickness / 2.0, (min_y + max_y) / 2.0, z_center, wall_thickness, max_y - min_y, height);
    add_wall((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, min_z - wall_thickness / 2.0, max_x - min_x, max_y - min_y, wall_thickness);
    add_wall((min_x + max_x) / 2.0, (min_y + max_y) / 2.0, max_z + wall_thickness / 2.0, max_x - min_x, max_y - min_y, wall_thickness);

    return workspace_obj;
}

visualization_msgs::msg::Marker CollisionObjectPublisher::create_workspace_marker() {
    visualization_msgs::msg::Marker workspace_marker;
    workspace_marker.header.frame_id = planning_frame_;
    workspace_marker.ns = "workspace";
    workspace_marker.id = 0;
    workspace_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    workspace_marker.action = visualization_msgs::msg::Marker::ADD;
    workspace_marker.scale.x = 0.02;  // line thickness
    workspace_marker.color.r = 0.0;
    workspace_marker.color.g = 1.0;  // green lines
    workspace_marker.color.b = 0.0;
    workspace_marker.color.a = 1.0;

    double min_z = workspace_.min_z;
    double min_x = workspace_.min_x;
    double min_y = workspace_.min_y;
    double max_x = workspace_.max_x;
    double max_z = workspace_.max_z;
    double max_y = workspace_.max_y;

    std::vector<std::array<double, 3>> corners = { { min_x, min_y, min_z }, { min_x, min_y, max_z },
    { min_x, max_y, min_z }, { min_x, max_y, max_z },
    { max_x, min_y, min_z }, { max_x, min_y, max_z },
    { max_x, max_y, min_z }, { max_x, max_y, max_z } };

    // Add edges
    auto add_edge = [&](int c1, int c2) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = corners[c1][0];
        p1.y = corners[c1][1];
        p1.z = corners[c1][2];
        p2.x = corners[c2][0];
        p2.y = corners[c2][1];
        p2.z = corners[c2][2];
        workspace_marker.points.push_back(p1);
        workspace_marker.points.push_back(p2);
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

    return workspace_marker;
}

moveit_msgs::msg::CollisionObject CollisionObjectPublisher::create_collision_object(CollisionObj col_obj) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.header.stamp = node_->now();
    collision_object.id = col_obj.name;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = { col_obj.width, col_obj.height, col_obj.depth };

    geometry_msgs::msg::Pose pose;
    pose.position.x = col_obj.x;
    pose.position.y = col_obj.y;
    pose.position.z = col_obj.z;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(pose);

    return collision_object;
}

visualization_msgs::msg::Marker CollisionObjectPublisher::create_collision_object_marker(CollisionObj col_obj) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.ns = col_obj.name;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = col_obj.width;
    marker.scale.y = col_obj.height; 
    marker.scale.z = col_obj.depth;
    marker.color.r = 1.0;  // red color
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.position.x = col_obj.x;
    marker.pose.position.y = col_obj.y;
    marker.pose.position.z = col_obj.z;
    marker.pose.orientation.w = 1.0;  // no rotation

    return marker;
}

void CollisionObjectPublisher::publish_collision_objects() {
    moveit_msgs::msg::PlanningSceneWorld psw;
    if (use_workspace_)
    {
      moveit_msgs::msg::CollisionObject workspace = create_workspace();
      psw.collision_objects.push_back(workspace);
    }
    for(auto col_obj : collision_objs_) {
        moveit_msgs::msg::CollisionObject col = create_collision_object(col_obj);
        psw.collision_objects.push_back(col);
    }
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;
    ps.world = psw;
    collision_objects_pub_->publish(ps);
}

void CollisionObjectPublisher::publish_markers() {
    // Publish it 10 times in case rviz hasn't started yet.
    for (int i = 0; i < 10; ++i) {
        if (use_workspace_)
        {
            auto marker = create_workspace_marker();
                marker_pub_->publish(marker);
            
        }

        for (auto & obj : collision_objs_)
        {
            auto col_marker = create_collision_object_marker(obj);
            marker_pub_->publish(col_marker);
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

} // namespace robco_validator