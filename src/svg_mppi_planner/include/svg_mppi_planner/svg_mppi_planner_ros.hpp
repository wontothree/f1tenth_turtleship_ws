// Anthony Garcia

#pragma once

#include <grid_map_ros/grid_map_ros.hpp>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include "svg_mppi_planner/svg_mppi.hpp"

// #include "example_interfaces/msg/float64.hpp"
// #include <std_msgs/msg/float64.hpp>

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
    const int timer_period = 10;                                  // timer period (ms)

private:
    std::unique_ptr<svg_mppi::planning::SVGMPPI> svg_mppi_pointer_;

    // subscribe
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr cost_map_subscriber_;

    // publish
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr steering_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_publisher_;

<<<<<<< HEAD
    // real system
    // rclcpp::Subscription<Float64>::SharedPtr steering_subscriber_;
=======
>>>>>>> 2ce29b16f16e5da0681063a06e28cc27225a4d4c
private:
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
    bool is_local_cost_map_received_ = false;
    grid_map::GridMap* local_cost_map_;
    void local_cost_map_callback(
        const grid_map_msgs::msg::GridMap::SharedPtr local_cost_map
    );

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
    bool is_odometry_received_ = false;
    void odometry_callback(
        const nav_msgs::msg::Odometry::SharedPtr odometry
    );

    /**
     * @brief make topic msg to visualize state sequence batch on rviz
     * @param state_sequence_batch state sequence batch to visualize
     * @param weight_batch
     */
    void visualize_state_sequence_batch(
        const planning::StateSequenceBatch& state_sequence_batch,
        const std::vector<double>& weight_batch);

    /**
     * @brief 
     * @param state_sequence state sequence to visualize
     * @param name_space name of the topic
     * @param rgb color of the topic (r, g, b)
     */
    void visualize_state_sequence(
        const planning::StateSequence& state_sequence,
        const std::string& name_space,
        const std::string& rgb
    );

    /**
     * @brief 
     * @param state_sequence state sequence to visualize
     * @param name_space name of the topic
     * @param rgb color of the topic (r, g, b)
     */
    void visualize_path(
        const planning::StateSequence& state_sequence,
        const std::string& name_space,
        const std::string& rgb
    );

    // // real system
    // void steering_callback(
    //     const Float64::SharedPtr steering
    // )
    // {
    //     robot_state_.steering = steering->data;

    //     std::cout << robot_state_.steering << std::endl;
    //     std::cout << "1" << std::endl;
    // };
};
}  // namespace svg_mppi
