// Anthony Garcia

#include <iostream>

#include "mppi_planner/mppi_base.hpp"

MPPIBase::MPPIBase()
{
    std::cout << "test" << std::endl;
}

/*
 * @brief set local cost map
 */
void MPPIBase::setLocalCostMap(const grid_map::GridMap& localCostMap) { localCostMap_ = localCostMap; }

/*
 * @brief set global cost map
 */
void MPPIBase::setGlobalCostMap(const grid_map::GridMap& globalCostMap) { globalCostMap_ = globalCostMap; }
