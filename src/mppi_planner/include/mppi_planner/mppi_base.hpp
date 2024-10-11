// Anthony Garcia

#pragma once

#include <grid_map_core/GridMap.hpp>

class MPPIBase
{
private:
    // local cost map object
    grid_map::GridMap localCostMap_;

    // global cost map object
    grid_map::GridMap globalCostMap_;

public:
    MPPIBase();

    /*
     * @brief set local cost map
     */
    void setLocalCostMap(const grid_map::GridMap& localCostMap);

    /*
     * @brief set global cost map
     */
    void setGlobalCostMap(const grid_map::GridMap& globalCostMap);
};
