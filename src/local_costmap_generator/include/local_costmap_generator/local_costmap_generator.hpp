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

class LocalCostmapGenerator : public rclcpp::Node 
{
private:
    // subscriber for scan topic
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;

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

public:
    LocalCostmapGenerator();

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void timerCallback();

    void sensorFrameToRobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl);
};
