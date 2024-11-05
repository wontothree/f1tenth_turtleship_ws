#include "mppi_planner/mppi_planner_ros.hpp"

namespace mppi {

MPPIPlannerROS::MPPIPlannerROS() : Node("local_planner_node")
{
    // flag for if received locql cost map
    isLocalCostMapReceived_ = false;

    // flag for if received global cost map
    isGlobalCostMapReceived_ = false;

    // flag for if received odometry
    isOdometryReceived_ = false;

    // initialize robotState
    robotState_.x = 0.0;
    robotState_.y = 0.0;
    robotState_.yaw = 0.0;
    robotState_.vel = 0.0;
    robotState_.steer = 0.0;

    // localizeless mode
    isLocalizeLessMode_ = true;

    // mppi solver pointor
    mppiSolverPtr_ = std::make_unique<mppi::trajectory::MPPI>();
}

/*
 * @brief called whenever received local cost map
 */
void MPPIPlannerROS::localCostMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr localCostMap)
{
    // Convert grid_map_msgs::msg::GridMap 2 grid_map::GridMap
    if (!grid_map::GridMapRosConverter::fromMessage(*localCostMap, localCostMap_)) {
        RCLCPP_ERROR(this->get_logger(), "[MPPIPlannerROS] Failed to convert grid_map_msgs::msg::GridMap to grid_map::GridMap");
        return;
    }

    // flag for if received local cost map
    isLocalCostMapReceived_ = true;
}

/*
 * @brief called whenever received global cost map
 */
void MPPIPlannerROS::globalCostMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr globalCostMap)
{
    if (!grid_map::GridMapRosConverter::fromMessage(*globalCostMap, globalCostMap_)) {
        RCLCPP_ERROR(this->get_logger(), "[MPPIPlannerROS] Failed to convert grid_map_msgs::msg::GridMap to grid_map::GridMap");
        return;
    }

    // flag for if received global cost map
    isGlobalCostMapReceived_ = true;
}

/*
 * @brief be called whenever received odometry
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
void MPPIPlannerROS::timerCallback()
{
    // status check
    if (isLocalizeLessMode_)
    {
        if (!isOdometryReceived_ || !isLocalCostMapReceived_)
        {
            RCLCPP_ERROR(this->get_logger(), "[MPPIPlannerROS] Not ready: Robot odometry: %d, Local cost map: %d", isOdometryReceived_, isLocalCostMapReceived_);
            return;
        }
    }
    else
    {
        if (!isOdometryReceived_ || !isLocalCostMapReceived_ || !isGlobalCostMapReceived_)
        {
            RCLCPP_ERROR(this->get_logger(), "[MPPIPlannerROS] Not ready: Robot odometry: %d, Local cost map: %d, Global cost map: %d", isOdometryReceived_, isLocalCostMapReceived_, isGlobalCostMapReceived_);
            return;
        }
    }

    // set local cost map
    mppiSolverPtr_->setLocalCostMap(localCostMap_);
    if(!isLocalizeLessMode_)
    {
        // set global cost map
        mppiSolverPtr_->setGlobalCostMap(globalCostMap_);
    }

    // steer observer

    // initialize state
    currentState_ = mppi::trajectory::State::Zero();
    currentState_[mppi::STATE_SPACE::x] = robotState_.x;
    currentState_[mppi::STATE_SPACE::y] = robotState_.y;
    currentState_[mppi::STATE_SPACE::yaw] = robotState_.yaw;
    currentState_[mppi::STATE_SPACE::vel] = robotState_.vel;
    currentState_[mppi::STATE_SPACE::steer] = robotState_.steer;

    // mppi solver
    const auto [optimalStateTrajectory_, optimalActionTrajectory_] = mppiSolverPtr_->solve(currentState_);
}

} // namespace mppi