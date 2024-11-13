// Anthony Garcia

#pragma once

<<<<<<< HEAD
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
=======
#include "rclcpp/rclcpp.hpp"
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
>>>>>>> ros codding

#include "rclcpp/rclcpp.hpp"
#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {

class SVGMPPIPlannerROS : public rclcpp::Node {
   public:
    SVGMPPIPlannerROS();
    ~SVGMPPIPlannerROS() {};

   private:
    const double gridLength = 20.0;                               // one side length of square cost map (m)
    const double resolution = 0.07;                               // resolution of cost map (m / grid)
    const std::string robot_frame_id_ = "ego_racecar/base_link";  // frame id of robot
    const std::string map_frame_id_ = "map";                      // frame id of map
    const int timer_period = 1000;                                // timer period (ms)

<<<<<<< HEAD
   private:
=======
>>>>>>> ros codding
    std::unique_ptr<svg_mppi::planning::SVGMPPI> svg_mppi_pointer_;

    // subscribe
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr cost_map_subscriber_;
<<<<<<< HEAD

    // publish
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_publisher_;
=======
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    
    // publish
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
    
private:
>>>>>>> ros codding
    /**
     * @brief This function is called in constant period of timer
     */
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    /**
     * @brief called whenever be subscribed by local cost map
     * @param local_cost_map
     */
<<<<<<< HEAD
    bool isLocalCostMapReceived_;
    grid_map::GridMap* local_cost_map_;
=======
    grid_map::GridMap* local_cost_map_;  
    bool is_local_cost_map_received_;
>>>>>>> ros codding
    void local_cost_map_callback(
        const grid_map_msgs::msg::GridMap::SharedPtr local_cost_map);

<<<<<<< HEAD
=======
    /**
     * @brief called whenever be subscribed by odometry
     */
    struct RobotState {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double velocity = 0.0;
        double steering = 0.0;
    };
    RobotState robot_state_;
    bool is_odometry_received_;
    void odometry_callback(
        const nav_msgs::msg::Odometry::SharedPtr odometry
    );

    //
>>>>>>> ros codding
    /**
     * @brief make topic msg to visualize state sequence batch on rviz
     * @param state_sequence_batch state sequence batch to visualize
     * @param weight_batch
     */
    void visualize_state_sequence_batch(
        const planning::StateSequenceBatch& state_sequence_batch,
        const std::vector<double>& weight_batch);

<<<<<<< HEAD
    /**
     * @brief 
     * @param state_sequence state sequence to visualize
     * @param name_space name of the topic
     * @param rgb color of the topic (r, g, b)
     */
    void visualize_state_sequence(
        const planning::StateSequence& state_sequence,
        const std::string& name_space,
        const std::string& rgb);

    /**
     * @brief 
     * @param state_sequence state sequence to visualize
     * @param name_space name of the topic
     * @param rgb color of the topic (r, g, b)
     */
    void visualize_path(
        const planning::StateSequence& state_sequence,
        const std::string& name_space,
        const std::string& rgb);
=======
    // /**
    //  * @brief 이진우 ref publish_traj
    //  */
    // void visualize_state_sequence(
    //     // state_sequence
    // );

    // /**
    //  * @brief 이진우 ref publish_path
    //  */
    // void visualize_path(
    //     // state_sequence
    // );
>>>>>>> ros codding
};
}  // namespace svg_mppi
