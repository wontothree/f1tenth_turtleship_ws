#include "local_planner/local_planner_ros.hpp"

MPPIPlannerROS::MPPIPlannerROS() : Node("local_planner_node")
{
    // flag for if received locql cost map
    isLocalCostMapReceived_ = false;

    // flag for if received odometry
    isOdometryReceived_ = false;

    // initialize robotState
    robotState_.x = 0.0;
    robotState_.y = 0.0;
    robotState_.yaw = 0.0;
    robotState_.vel = 0.0;
    robotState_.steer = 0.0;
}

/*
 * be called whenever received local cost map
 */
void MPPIPlannerROS::localCostMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr localCostMap)
{
    // Convert grid map message to internal representation
    // if (!grid_map::GridMapRosConverter::fromMessage(*localCostMap, obstacle_map_)) {
    //     RCLCPP_ERROR(this->get_logger(), "[MPPIPlannerROS] Failed to convert grid map to grid map");
    //     return;
    // }

    // flag for if received local cost map
    isLocalCostMapReceived_ = true;
}

/*
 *  be called whenever received odometry
 */ 
void MPPIPlannerROS::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    // update robot state
    robotState_.x = 0.0;
    robotState_.y = 0.0;
    robotState_.yaw = 0.0;
    robotState_.vel = odometry->twist.twist.linear.x;

    // flag for if received odometry
    isOdometryReceived_ = true;
}

/*
 * be called in constant period of timer
 */
// void MPPIPlannerROS::timerCallback()
// {

// }