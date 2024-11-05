// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/odometry.hpp>  

#include "mppi_planner/mppi_types.hpp"
#include "mppi_planner/mppi_template.hpp"
#include "mppi_planner/mppi.hpp"

namespace mppi {

class MPPIPlannerROS : public rclcpp::Node
{
private:
    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    /*
     * be called whenever received local cost map
     */
    // flag for if received local cost map
    bool isLocalCostMapReceived_;
    grid_map::GridMap localCostMap_;     
    void localCostMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr localCostMap); 

    /*
     * @brief be called if received global cost map
     */
    bool isGlobalCostMapReceived_;
    grid_map::GridMap globalCostMap_;
    void globalCostMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr globalCostMap);
    
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


    // flag for localizeless mode
    bool isLocalizeLessMode_;
    // mppi solver pointer
    std::unique_ptr<mppi::trajectory::MPPITemplate> mppiSolverPtr_;
    // current state
    mppi::trajectory::State currentState_;
    /*
     * be called in constant period of timer
     */
    void timerCallback();

public:
    MPPIPlannerROS();
    ~MPPIPlannerROS() {};
};

} // namespace mppi