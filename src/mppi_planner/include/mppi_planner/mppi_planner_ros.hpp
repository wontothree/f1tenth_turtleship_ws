// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include "mppi_types.hpp"

class MPPIPlannerROS : public rclcpp::Node
{
public:
    MPPIPlannerROS();
    ~MPPIPlannerROS() {};

private:
    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    /*
     * be called whenever received local cost map
     */
    // flag for if received local cost map
    bool isLocalCostMapReceived_;     
    void localCostMapCallback(const grid_map::msg::GridMap::SharedPtr localCostMap);
    
    /*
     * be called whenever received odometry
     */
    // robot state struecture
    struct RobotState {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double vel = 0.0;
        double steer = 0.0;
    };
    RobotState robotState_;
    bool isOdometryReceived_;
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);

    /*
     * be called in constant period of timer
     */
    // void timerCallback();
};