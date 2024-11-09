#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"

#include <grid_map_ros/grid_map_ros.hpp>

class LocalCostmapGenerator : public rclcpp::Node 
{
private:
    //// constants: 이곳의 상수들을 수정하여 grid map의 크기, 해상도, 업데이트 주기, 
    //// robot frame(base frame) 이름, sensor frame(laser frame) 이름을 조정할 수 있다.

    // grid map 의 인덱스 개수. 이는 n x n 2D grid map에서 n을 의미한다.
    const int gridSize = 40;

    const double gridLength = 6.0; // grid map의 길이 (meter)

    const double resolution = gridLength / gridSize; // grid map의 해상도 (meter)

    // grid map을 업데이트하는 주기(ms)
    const int timerPeriod = 1000; 

    // frame id
    const std::string robotFrameId_ = "ego_racecar/base_link";
    const std::string sensorFrameId_ = "ego_racecar/laser";
    
    //// variables:

    // subscriber for scan topic
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;

    // publisher for costmap
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr costmapPublisher_;

    // publisher for occupancy grid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapPublisher2_;

    // flag for if received scan
    bool isScanReceived_;

    // timer for function timerCallback
    rclcpp::TimerBase::SharedPtr timer_;

    // laser projection object used for method `projectLaser`
    std::shared_ptr<laser_geometry::LaserProjection> laserProjection_;

    // pointcloud2 object
    std::shared_ptr<sensor_msgs::msg::PointCloud2> pointCloud2_;

    // pcl object
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_;
    
    // transform stamped object
    geometry_msgs::msg::TransformStamped transformStamped_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // grid map object
    grid_map::GridMap* costmap_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void sensorFrameToRobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl);

    void timerCallback();

    std::vector<grid_map::Index> pclToCostmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl, grid_map::GridMap* costmap) const;

public:
    LocalCostmapGenerator();
};
