#include "local_costmap_generator/local_costmap_generator.hpp"

LocalCostmapGenerator::LocalCostmapGenerator() : Node("local_costmap_generator_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // subscriber for laser scan
    scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LocalCostmapGenerator::scanCallback, this, std::placeholders::_1)
    );

    // publisher for costmap
    costmapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("costmap_topic", 10);

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

    // initialize costmap
    grid_map::GridMap* costmap_ = new grid_map::GridMap();
    costmap_->add("collision_layer", grid_map::Matrix::Zero(costmap_->getSize()(0), costmap_->getSize()(1)));
    costmap_->setGeometry(grid_map::Length(10.0, 10.0), 0.1); // length: 10m, weight: 10m, resolution: 0.1m
    // 모든 값 0으로 초기화
    for (const auto& layer : costmap_->getLayers()) {
        grid_map::Matrix& data = costmap_->get(layer);
        data.setZero(); // 모든 값을 0으로 설정
    }
    // std::cout << "GridMap Information:" << std::endl;
    
    // // 해상도 출력
    // std::cout << "Resolution: " << costmap_->getResolution() << " m" << std::endl;

    // // 길이 출력
    // std::cout << "Length: " << costmap_->getLength().transpose() << " m" << std::endl;

    // // 위치 출력
    // std::cout << "Position: " << costmap_->getPosition().transpose() << std::endl;

    // // 크기 출력
    // std::cout << "Size: " << costmap_->getSize().transpose() << std::endl;

    // // 레이어 정보 출력
    // std::cout << "Layers: ";
    // const auto& layers = costmap_->getLayers();
    // for (const auto& layer : layers) {
    //     std::cout << layer << " ";
    // }
    // std::cout << std::endl;

    // // 충돌 레이어 데이터 출력
    // if (costmap_->exists("collision_layer")) {
    //     const grid_map::Matrix& collisionLayer = costmap_->get("collision_layer");
    //     std::cout << "Collision Layer Data:\n" << collisionLayer << std::endl;
    // } else {
    //     std::cout << "No collision layer data available." << std::endl;
    // }
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
    // std::stringstream ss;
    // ss << "PointCloud contains " << pcl_->size() << " points:\n";
    // for (const auto& point : pcl_->points) {
    //     ss << "Point: [x: " << point.x << ", y: " << point.y << ", z: " << point.z << "]\n";
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    // remove cloud points within robot ----------------------------------------------------------

    // convert pcl to costmap
    // const std::vector<grid_map::Index> occupiedIndices = pclToCostmap(pcl_, costmap_);
    std::cout << "test" << std::endl;

    // Output the occupied indices
    // std::cout << "Occupied Indices:" << std::endl;
    // for (const auto& index : occupiedIndices) {
    //     std::cout << "(" << index(0) << ", " << index(1) << ")" << std::endl;
    // }

    // inflate rigid body -------------------------------------------------------------------------

    // pcl::PointCloud 2 sensor_msgs::PointCloud2

    // publish
    // grid_map_msgs::msg::GridMap costmapMsg;
    // costmap_->toMessage(costmapMsg); // costmap에서 메시지를 가져옵니다.
    // costmapPublisher_->publish(costmapMsg); // GridMap 메시지를 퍼블리시합니다.
}

void LocalCostmapGenerator::sensorFrameToRobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl)
{   
    /*
        transform senfor frame to robot frame
    */
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

// std::vector<grid_map::Index> LocalCostmapGenerator::pclToCostmap(
//     const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl,
//     grid_map::GridMap* costmap) const
// {
//     // // initialize collision layer
//     // grid_map::Matrix& costmapData = costmap->get("collision_layer");
//     // costmapData.setZero();
    
//     // // initialize
//     // std::vector<grid_map::Index> occupiedIndices(pcl->points.size());


//     std::cout << "2" << std::endl;

//     // transform pcl to costmap using parallel processing
//     // #pragma omp parallel for num_threads(8)
//     // for (unsigned int i = 0; i < pcl->points.size(); ++i) {
//     //     const auto& point = pcl->points[i];

//     //     std::cout << "1" << std::endl;

//     //     // 포인트가 비용 맵 내부에 있을 경우 처리
//     //     if (costmap->isInside(grid_map::Position(point.x, point.y))) {
//     //         std::cout << "in" << std::endl;
//     //         // grid_map::Index index;
//     //         // costmap->getIndex(grid_map::Position(point.x, point.y), index);
//     //         // costmapData(index.x(), index.y()) = 100; // 사용자 정의 constant
            
//     //         // occupiedIndices[i] = index; // 인덱스를 추가
//     //     } else {
//     //         occupiedIndices[i] = grid_map::Index(-1, -1); // 범위를 벗어난 경우
//     //     }
//     // }

//     // // (-1, -1) 인덱스를 제거하여 최종 occupied_indices를 만듭니다.
//     // occupiedIndices.erase(std::remove_if(occupiedIndices.begin(), occupiedIndices.end(),
//     //                                       [](const grid_map::Index& index) { return index.x() == -1 && index.y() == -1; }),
//     //                        occupiedIndices.end());

//     return occupiedIndices;
// }
