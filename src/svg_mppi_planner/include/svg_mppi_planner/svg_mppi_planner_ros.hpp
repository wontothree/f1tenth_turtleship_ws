// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {

class SVGMPPIPlannerROS : public rclcpp::Node
{
public:
    SVGMPPIPlannerROS();
    ~SVGMPPIPlannerROS() {};

private:
    const double gridLength = 20.0;                     // one side length of sqaure cost map (m)
    const double resolution = 0.07;                     // resolution of cost map (m / grid)

private:
    std::unique_ptr<svg_mppi::planning::SVGMPPI> svg_mppi_pointer_;

    // subscribe
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr cost_map_subscriber_;

    /**
    * @brief be called in constant period of timer
    */
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    /**
     * @brief be called whenever be substribed by local cost map
     */
    bool isLocalCostMapReceived_;
    grid_map::GridMap* local_cost_map_;  
    void local_cost_map_callback(
        const grid_map_msgs::msg::GridMap::SharedPtr local_cost_map
    );

    /**
     * @brief 이진우
     */
    void visualize_state_sequence_batch(
        // state_sequence_batch
        // weight_batch
        // publisher
    )

    /**
     * @brief 이진우 ref publish_traj
     */
    void visualize_state_sequence(
        // state_sequence
    )

    /**
     * @brief 이진우 ref publish_path
     */
    void visualize_path(
        // state_sequence
    )
};
}