// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {

class SVGMPPIPlannerROS : public rclcpp::Node
{
private:
    std::unique_ptr<svg_mppi::planning::SVGMPPI> svg_mppi_pointer_;

    // subscribe
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr cost_map_subscriber_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    /*
    * be called in constant period of timer
    */
    void timer_callback();

    bool isLocalCostMapReceived_;
    grid_map::GridMap* local_cost_map_;  
    void local_cost_map_callback(
        const grid_map_msgs::msg::GridMap::SharedPtr local_cost_map
    );

public:
    SVGMPPIPlannerROS();
    ~SVGMPPIPlannerROS() {};
};
}