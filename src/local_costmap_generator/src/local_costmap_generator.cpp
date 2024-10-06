#include "local_costmap_generator/local_costmap_generator.hpp"

LocalCostmapGenerator::LocalCostmapGenerator() : Node("local_costmap_generator_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LocalCostmapGenerator::scanCallback, this, std::placeholders::_1)
    );

    // flag for if received scan
    isScanReceived_ = false;

    // timer_callback
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LocalCostmapGenerator::timerCallback, this));

    // LaserProjection Object used for method `projectLaser`
    laserProjection_ = std::make_shared<laser_geometry::LaserProjection>();

    // pointcloud2 object
    pointCloud2_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // pcl object
    pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    
}

void LocalCostmapGenerator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // transform scan to pointcloud2
    laserProjection_->projectLaser(*scan, *pointCloud2_);

    // // print data point
    // std::cout << "Data Points:" << std::endl;
    // const float* pointData = reinterpret_cast<const float*>(&pointCloud2_->data[0]); // 데이터 포인터를 float로 변환

    // size_t pointStep = pointCloud2_->point_step / sizeof(float); // 포인트 스텝에 따른 포인트 수

    // for (size_t i = 0; i < pointCloud2_->width; ++i) {
    //     std::cout << "Point " << i << ": (";
    //     for (size_t j = 0; j < pointStep; ++j) {
    //         std::cout << pointData[i * pointStep + j];
    //         if (j < pointStep - 1) {
    //             std::cout << ", ";
    //         }
    //     }
    //     std::cout << ")" << std::endl;
    // }

    // transform pointcloud2 to pcl
    pcl::fromROSMsg(*pointCloud2_, *pcl_);

    // // 포인트 클라우드의 포인트 수를 출력합니다.
    // std::cout << "Number of points: " << pcl_->points.size() << std::endl;

    // // 포인트 클라우드의 각 포인트를 출력합니다.
    // std::cout << "Data Points:" << std::endl;
    // for (size_t i = 0; i < pcl_->points.size(); ++i) {
    //     const pcl::PointXYZ& point = pcl_->points[i]; // 포인트에 접근
    //     std::cout << "Point " << i << ": (" 
    //               << point.x << ", " 
    //               << point.y << ", " 
    //               << point.z << ")" << std::endl; // x, y, z 좌표 출력
    // }

    isScanReceived_ = true;
}

void LocalCostmapGenerator::timerCallback()
{
    if (!isScanReceived_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
        return;
    }

    // preprocess

    // sensor frame coordinate 2 robot frame coordinate
    sensorFrameToRobotFrame(pcl_);

    // pcl_의 포인트 개수와 각 포인트의 좌표를 출력
    std::cout << "test\n" << std::endl;

    // std::stringstream ss;
    // ss << "PointCloud contains " << pcl_->size() << " points:\n";
    // for (const auto& point : pcl_->points) {
    //     ss << "Point: [x: " << point.x << ", y: " << point.y << ", z: " << point.z << "]\n";
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());


    // 5. remove cloud points within robot

    // 6. pcl 2 costmap

    // 7. inflate rigid body

    // pcl::PointCloud 2 sensor_msgs::PointCloud2

    // publish
}

void LocalCostmapGenerator::sensorFrameToRobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl)
{
    std::string robotFrameId_ = "ego_racecar/base_link";
    std::string sensorFrameId_ = "ego_racecar/laser";

    try {
        transformStamped_ = tf_buffer_.lookupTransform(robotFrameId_, sensorFrameId_, rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[LocalCostmapGenerator] %s", ex.what());
        return;
    }

    const Eigen::Isometry3d transformMatrix = tf2::transformToEigen(transformStamped_.transform);

    pcl::transformPointCloud(*pcl, *pcl, transformMatrix.matrix().cast<float>());
}
