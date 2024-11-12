#include "svg_mppi_planner/svg_mppi_planner_ros.hpp"

namespace svg_mppi { 

SVGMPPIPlannerROS::SVGMPPIPlannerROS() : Node("svg_mppi_planner_node")
{
    svg_mppi_pointer_ = std::make_unique<svg_mppi::planning::SVGMPPI>();

    cost_map_subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "costmap_topic",
        10, 
        std::bind(&SVGMPPIPlannerROS::local_cost_map_callback, this, std::placeholders::_1)
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

}