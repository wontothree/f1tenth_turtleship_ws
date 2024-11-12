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
    const double gridLength = 20.0;                     // one side length of sqaure cost map (m)
    const double resolution = 0.07;                     // resolution of cost map (m/grid)
    const int gridSize = gridLength / resolution;       // number of index in cost map 

    const int timerPeriod = 10;                       // period of updating cost map (ms)

    // frame id
    const std::string robotFrameId_ = "ego_racecar/base_link";
    const std::string sensorFrameId_ = "ego_racecar/laser";
    
    //// variables:

    // subscriber for scan topic
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;

    // publisher for costmap
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr costmapPublisher_;

    // publisher for occupancy grid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancyGridPublisher_;

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

    /**
     * @brief scan 토픽이 발행되었을 때 호출되는 콜백함수. 
        scan 토픽 정보를 받아 pointcloud2로 변환하고, 이를 pcl로 변환한다.
    * 
    * @param scan : scan 
    */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief 특정 주기마다 호출되어 grid map을 업데이트하고, costmap 토픽을 발행한다.
        costmap은 두가지 타입으로 발행한다. 하나는 grid_map_msgs::msg::GridMap, 다른 하나는 nav_msgs::msg::OccupancyGrid이다.
    * 
    */
    void timerCallback();

    /**
     * @brief pcl을 받아, 그 좌표의 기준을 laser frame에서 base frame으로 변환한다. 
        따라서 이것은 반드시 '/tf_static'이 laser frame과 base frame 사이의 변환을 포함해야 동작한다.
    * @param pcl pcl::PointXYZ 타입의 포인터
    */
    void sensorFrameToRobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl);

    /**
     * @brief pcl을 받아, 그 좌표를 costmap의 좌표로 변환한다.
     * @param pcl pcl::PointXYZ 타입의 포인터
     * @param costmap grid_map::GridMap 타입의 포인터
     * @return std::vector<grid_map::Index> 변환된 좌표의 인덱스
     */
    std::vector<grid_map::Index> pclToCostmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl, grid_map::GridMap* costmap) const;

public:
    /**
     * @brief 
        이 노드는 scan 토픽을 구독하고, costmap 토픽을 발행한다.
        costmap은 두가지 타입으로 발행한다. 하나는 grid_map_msgs::msg::GridMap, 다른 하나는 nav_msgs::msg::OccupancyGrid이다.
        grid_map_msgs::msg::GridMap은 grid_map_msgs 패키지에서 제공하는 메시지 타입이다. 이는 grid map을 표현하는 메시지이다. 
    * 
    */
    LocalCostmapGenerator();
};
