#include "svg_mppi_planner/svg_mppi_planner_ros.hpp"

namespace svg_mppi { 

SVGMPPIPlannerROS::SVGMPPIPlannerROS() : Node("svg_mppi_planner_node")
{
    svg_mppi_pointer_ = std::make_unique<svg_mppi::planning::SVGMPPI>();

    // initialize local cost map
    local_cost_map_ = new grid_map::GridMap({"collision_layer"});
    local_cost_map_->setGeometry(grid_map::Length(gridLength, gridLength), resolution, grid_map::Position(0.0, 0.0));
    local_cost_map_->get("collision_layer").setConstant(0.0);

    cost_map_subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "cost_map",
        10, 
        std::bind(&SVGMPPIPlannerROS::local_cost_map_callback, this, std::placeholders::_1)
    );

    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "candidate_paths",
        10
    );
}

void SVGMPPIPlannerROS::timer_callback()
{
    //
}

void SVGMPPIPlannerROS::local_cost_map_callback(
    const grid_map_msgs::msg::GridMap::SharedPtr local_cost_map
)
{
    // Convert grid_map_msgs::msg::GridMap 2 grid_map::GridMap
    if (!grid_map::GridMapRosConverter::fromMessage(*local_cost_map, *local_cost_map_)) {
        RCLCPP_ERROR(this->get_logger(), "[MPPIPlannerROS] Failed to convert grid_map_msgs::msg::GridMap to grid_map::GridMap");
        return;
    }

    // flag for if received local cost map
    isLocalCostMapReceived_ = true;

    Eigen::MatrixXf& costmapData = local_cost_map_->get("collision_layer");
    std::cout << "----------------------------------" << std::endl;
    std::cout << costmapData << std::endl;
    std::cout << "----------------------------------" << std::endl;
}

void SVGMPPIPlannerROS::visualize_state_sequence_batch(
    const planning::StateSequenceBatch& state_sequence_batch,
    const std::vector<double>& weight_batch
){
    assert(state_sequence_batch.size() == weight_batch.size());

    visualization_msgs::msg::MarkerArray marker_array;

    const double max_weight = weight_batch[0];
    const double max_node_size = 0.05;
    for (size_t i = 0; i < state_sequence_batch.size(); i++) {
        visualization_msgs::msg::Marker line;

        line.header.frame_id = robot_frame_id_;
        line.header.stamp = this->get_clock()->now();
        line.ns = "candidate_path_line";
        line.id = i;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.x = 0.0;
        line.pose.orientation.y = 0.0;
        line.pose.orientation.z = 0.0;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.01;
        line.color.a = 0.3;
        line.color.r = 0.0;
        line.color.g = 0.5;
        line.color.b = 1.0;
        // line.lifetime = ros::Duration(0.1);

        // nodes
        visualization_msgs::msg::Marker nodes;

        nodes.header.frame_id = robot_frame_id_;
        nodes.header.stamp = this->get_clock()->now();
        nodes.ns = "candidate_path_nodes";
        nodes.id = line.id + 10000 + i;
        nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes.action = visualization_msgs::msg::Marker::ADD;
        nodes.pose.orientation.x = 0.0;
        nodes.pose.orientation.y = 0.0;
        nodes.pose.orientation.z = 0.0;
        nodes.pose.orientation.w = 1.0;
        nodes.scale.x = weight_batch[i] * max_node_size / max_weight + 0.01;
        nodes.scale.y = 0.01;
        nodes.scale.z = 0.01;
        nodes.color.a = 0.6;
        nodes.color.r = 0.0;
        nodes.color.g = 0.5;
        nodes.color.b = 1.0;
        // nodes.lifetime = ros::Duration(0.1);

        for (int j = 0; j < state_sequence_batch.at(0).rows(); j++) {
            geometry_msgs::msg::Point point;
            point.x = state_sequence_batch.at(i)(j, STATE_SPACE::x);
            point.y = state_sequence_batch.at(i)(j, STATE_SPACE::y);
            point.z = 0.0;
            line.points.push_back(point);
            nodes.points.push_back(point);
        }
        marker_array.markers.push_back(line);
        marker_array.markers.push_back(nodes);
    }

    marker_array_publisher_->publish(marker_array);

}

}