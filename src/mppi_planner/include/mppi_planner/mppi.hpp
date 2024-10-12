// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <grid_map_core/GridMap.hpp>

#include "mppi_planner/mppi_template.hpp"
#include "mppi_planner/mppi_base.hpp"

namespace mppi {
    
namespace cpu {

/*
 * @brief Model Path Integral Control
 */
class MPPI: public rclcpp::Node, public MPPITemplate
{
private:
    std::unique_ptr<MPPIBase> mppiBasePtr_;

public:
    MPPI();

    /*
     * @brief solve mppi problem
     * @param current state
     * @return optimal state trajectory and optimal action trajectory
     */
    std::pair<StateTrajectory, ActionTrajectory> solve(const State& currentState);

    /*
     * @brief set local cost map
     */
    void setLocalCostMap(const grid_map::GridMap& localCostMap);

    /*
     * @brief set global cost map
     */
    void setGlobalCostMap(const grid_map::GridMap& globalCostMap);
};

} // namespace cpu

} // namespace mppi
