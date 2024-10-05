#include "local_costmap_generator/local_costmap_generator.hpp"

LocalCostmapGenerator::LocalCostmapGenerator() : Node("local_costmap_generator_node")
{
    scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LocalCostmapGenerator::scanCallback, this, std::placeholders::_1)
    );
}

void LocalCostmapGenerator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    std::cout << scan << std::endl;
}
