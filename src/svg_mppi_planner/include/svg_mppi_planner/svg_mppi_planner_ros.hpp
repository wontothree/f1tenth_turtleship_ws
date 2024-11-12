// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {

class SVGMPPIPlannerROS : public rclcpp::Node
{
public:
    SVGMPPIPlannerROS();
    ~SVGMPPIPlannerROS() {};

private:
    const double gridLength = 20.0;                     // one side length of square cost map (m)
    const double resolution = 0.07;                     // resolution of cost map (m / grid)
    const std::string robot_frame_id_ = "ego_racecar/base_link";    // frame id of robot
    const std::string map_frame_id_ = "map";            // frame id of map
    const int timer_period = 1000;                      // timer period (ms)

private:
    std::unique_ptr<svg_mppi::planning::SVGMPPI> svg_mppi_pointer_;

    // subscribe
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr cost_map_subscriber_;
    
    // publish
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
    
    /**
    * @brief This function is called in constant period of timer
    */
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    /**
     * @brief This function is called whenever be subscribed by local cost map
     */
    bool isLocalCostMapReceived_;
    grid_map::GridMap* local_cost_map_;  
    void local_cost_map_callback(
        const grid_map_msgs::msg::GridMap::SharedPtr local_cost_map
    );

    // (이진우)working...
    /**
     * @brief make topic msg to visualize state sequence batch on rviz
     * @param state_sequence_batch state sequence batch to visualize
     * @param weight_batch 
     */
    void visualize_state_sequence_batch(
        const planning::StateSequenceBatch& state_sequence_batch,
        const std::vector<double>& weight_batch
    );
}; // class SVGMPPIPlannerROS
} // namespace svg_mppi