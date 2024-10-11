// Anthony Garcia

#pragma once

#include <utility>
#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>

#include "local_planner/mpc_type.hpp"

class MPPITemplate 
{
public:
    virtual ~MPPITemplate() = default;

    /*
     * @brief mppi solver
     */
    //

    /* 
     * @brief setter pure virtual function for local cost map
     */
    virtual void setLocalCostMap(const grid_map::GridMap& localCostMap) = 0;

    /*
     * @brief setter function for global cost map
     */
    virtual void setGlobalCostMap(const grid_map::GridMap& globalCostMap) = 0;

private:
}
