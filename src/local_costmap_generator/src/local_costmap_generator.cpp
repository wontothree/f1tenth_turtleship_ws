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
    // // 모든 값 0으로 초기화
    // for (const auto& layer : costmap_->getLayers()) {
    //     grid_map::Matrix& data = costmap_->get(layer);
    //     data.setZero(); // 모든 값을 0으로 설정
    // }
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
    // print
    // std::cout << "PointCloud contains " << pcl_->size() << " points:\n" << std::endl;

    // for (const auto& point : pcl_->points) {
    //     std::cout << "Point: [x: " << point.x << ", y: " << point.y << ", z: " << point.z << "]\n" << std::endl;
    // }

    // remove cloud points within robot ----------------------------------------------------------

    // convert pcl to costmap
    const std::vector<grid_map::Index> occupiedIndices = pclToCostmap(pcl_, costmap_);

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

        this function need topic '/tf_static'
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
//     grid_map::GridMap* costmap
// ) const
// {
//     // PointCloud의 크기를 기반으로 occupiedIndices 초기화
//     std::vector<grid_map::Index> occupiedIndices(pcl->points.size(), grid_map::Index(0, 0));

//     for (unsigned int index = 0; index < pcl->points.size(); ++index) {
//         const auto& point = pcl->points[index];  // 잘못된 변수명 수정

//         // 포인트가 costmap 내에 있는지 확인
//         if (point.x >= -2.0 && point.x <= 2.0 && point.y >= -2.0 && point.y <= 2.0) {
//             // 그리드 인덱스를 계산 (포인트 클라우드 좌표를 그리드 인덱스에 매핑)
//             int grid_x = static_cast<int>((point.x + 2.0) / 0.1); // x 좌표를 그리드 인덱스로 변환
//             int grid_y = static_cast<int>((point.y + 2.0) / 0.1); // y 좌표를 그리드 인덱스로 변환

//             // 변환된 인덱스를 occupiedIndices에 저장
//             occupiedIndices[index] = grid_map::Index(grid_x, grid_y);
//         } else {
//             // 포인트가 costmap 바깥에 있는 경우 (-1, -1)로 표시
//             occupiedIndices[index] = grid_map::Index(-1, -1);
//         }
//     }

//     // 디버깅용 출력
//     for (size_t i = 0; i < occupiedIndices.size(); ++i) {
//         std::cout << "Index " << i << ": (" << occupiedIndices[i].x() << ", " << occupiedIndices[i].y() << ")" << std::endl;
//     }


//     return occupiedIndices;
// }

// std::vector<grid_map::Index> LocalCostmapGenerator::pclToCostmap(
//     const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl,
//     grid_map::GridMap* costmap
// ) const
// {
//     std::vector<grid_map::Index> occupiedIndices(pcl->points.size(), grid_map::Index(0, 0));
    
//     // 40x40 행렬 초기화
//     const int gridSize = 40;
//     std::vector<std::vector<int>> visualMatrix(gridSize, std::vector<int>(gridSize, 0));

//     for (unsigned int index = 0; index < pcl->points.size(); ++index) {
//         const auto& point = pcl->points[index];

//         if (point.x >= -2.0 && point.x <= 2.0 && point.y >= -2.0 && point.y <= 2.0) {
//             int grid_x = static_cast<int>((point.x + 2.0) / 0.1); // x 좌표를 그리드 인덱스로 변환
//             int grid_y = static_cast<int>((point.y + 2.0) / 0.1); // y 좌표를 그리드 인덱스로 변환

//             // 변환된 인덱스를 occupiedIndices에 저장
//             occupiedIndices[index] = grid_map::Index(grid_x, grid_y);
            
//             // 2D 행렬에 해당 인덱스를 1로 설정
//             visualMatrix[grid_y][grid_x] = 1; // y 방향이 아래쪽, x 방향이 오른쪽
//         } else {
//             occupiedIndices[index] = grid_map::Index(-1, -1);
//         }
//     }

//     // 디버깅용 출력: 40x40 행렬 시각화
//     for (int y = gridSize - 1; y >= 0; --y) { // y 방향이 아래쪽이므로 역순으로 출력
//         for (int x = 0; x < gridSize; ++x) {
//             std::cout << visualMatrix[y][x];
//         }
//         std::cout << std::endl;
//     }
//     std::cout << "----------------------------------" << std::endl;

//     return occupiedIndices;
// }
std::vector<grid_map::Index> LocalCostmapGenerator::pclToCostmap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl,
    grid_map::GridMap* costmap
) const
{
    std::vector<grid_map::Index> occupiedIndices(pcl->points.size(), grid_map::Index(0, 0));
    
    // 40x40 행렬 초기화
    const int gridSize = 40;
    std::vector<std::vector<int>> visualMatrix(gridSize, std::vector<int>(gridSize, 0));

    // 정 중앙을 5로 표시
    int centerX = gridSize / 2; // 중앙 x 인덱스
    int centerY = gridSize / 2; // 중앙 y 인덱스
    visualMatrix[centerY][centerX] = 5; // 중앙을 5로 설정

    for (unsigned int index = 0; index < pcl->points.size(); ++index) {
        const auto& point = pcl->points[index];

        if (point.x >= -2.0 && point.x <= 2.0 && point.y >= -2.0 && point.y <= 2.0) {
            int grid_x = static_cast<int>((point.x + 2.0) / 0.1); // x 좌표를 그리드 인덱스로 변환
            int grid_y = static_cast<int>((point.y + 2.0) / 0.1); // y 좌표를 그리드 인덱스로 변환

            // 그리드 인덱스가 범위를 벗어나지 않도록 체크
            if (grid_x >= 0 && grid_x < gridSize && grid_y >= 0 && grid_y < gridSize) {
                occupiedIndices[index] = grid_map::Index(grid_x, grid_y);
                visualMatrix[grid_y][grid_x] = 1; // 2D 행렬에 해당 인덱스를 1로 설정
            } else {
                occupiedIndices[index] = grid_map::Index(-1, -1); // 범위를 벗어난 인덱스
            }
        } else {
            occupiedIndices[index] = grid_map::Index(-1, -1);
        }
    }

    // 디버깅용 출력: 40x40 행렬 시각화
    std::cout << "----------------------------------" << std::endl;
    for (int y = gridSize - 1; y >= 0; --y) { // y 방향이 아래쪽이므로 역순으로 출력
        for (int x = 0; x < gridSize; ++x) {
            std::cout << visualMatrix[y][x] << " "; // 공백 추가
        }
        std::cout << std::endl;
    }
    std::cout << "----------------------------------" << std::endl;

    return occupiedIndices;
}
