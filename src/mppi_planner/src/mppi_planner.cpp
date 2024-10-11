#include "mppi_planner/mppi_planner.hpp"

MPPIPlanner::MPPIPlanner() : Node("mppi_planner_node")
{
    std::cout << "test" << std::endl;
}

void MPPIPlanner::setLocalCostMap(const grid_map::GridMap& localCostMap) { mppiBasePtr_->setLocalCostMap(localCostMap); }

void MPPIPlanner::setGlobalCostMap(const grid_map::GridMap& globalCostMap) { mppiBasePtr_->setGlobalCostMap(globalCostMap); }
