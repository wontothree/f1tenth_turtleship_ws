#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

class LocalCostmapGenerator : public rclcpp::Node 
{
private:
    // subscriber for scan topic
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;

    // LaserProjection Object used for method `projectLaser`
    std::shared_ptr<laser_geometry::LaserProjection> laserProjection_;

public:
    LocalCostmapGenerator();

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};
