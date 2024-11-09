#include "local_costmap_generator/local_costmap_generator.hpp"


/**
 * @brief LocalCostmapGenerator의 생성자. 
    이 노드는 scan 토픽을 구독하고, costmap 토픽을 발행한다.
    costmap은 두가지 타입으로 발행한다. 하나는 grid_map_msgs::msg::GridMap, 다른 하나는 nav_msgs::msg::OccupancyGrid이다.
    grid_map_msgs::msg::GridMap은 grid_map_msgs 패키지에서 제공하는 메시지 타입이다. 이는 grid map을 표현하는 메시지이다. 
 * 
 */
LocalCostmapGenerator::LocalCostmapGenerator() : Node("local_costmap_generator_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // subscriber for laser scan
    scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LocalCostmapGenerator::scanCallback, this, std::placeholders::_1)
    );

    // publisher for costmap
    costmapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("costmap_topic", 10);
    costmapPublisher2_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap_topic_oc", 10);
    
    // flag for if received scan
    isScanReceived_ = false;

    // timer_callback함수를 timerPeriod(ms)마다 호출한다.
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timerPeriod), std::bind(&LocalCostmapGenerator::timerCallback, this));

    // LaserProjection Object used for method `projectLaser`
    laserProjection_ = std::make_shared<laser_geometry::LaserProjection>();

    // pointcloud2 object
    pointCloud2_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // pcl object
    pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // grid map object
    costmap_ = new grid_map::GridMap({"collision_layer"});
    costmap_->setGeometry(grid_map::Length(gridLength, gridLength), resolution, grid_map::Position(0.0, 0.0));
    costmap_->get("collision_layer").setConstant(0.0);
}

/**
 * @brief scan 토픽이 발행되었을 때 호출되는 콜백함수. 
    scan 토픽 정보를 받아 pointcloud2로 변환하고, 이를 pcl로 변환한다.
 * 
 * @param scan : scan 
 */
void LocalCostmapGenerator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // scan 데이터를 pointcloud2로 변환한다.
    laserProjection_->projectLaser(*scan, *pointCloud2_);

    // 데이터 전처리를 위해 pointcloud2를 pcl로 변환한다.
    pcl::fromROSMsg(*pointCloud2_, *pcl_);

    // 스캔 데이터를 받았다는 플래그를 true로 설정한다.
    isScanReceived_ = true;
}

/**
 * @brief 특정 주기마다 호출되어 grid map을 업데이트하고, costmap 토픽을 발행한다.
    costmap은 두가지 타입으로 발행한다. 하나는 grid_map_msgs::msg::GridMap, 다른 하나는 nav_msgs::msg::OccupancyGrid이다.
 * 
 */
void LocalCostmapGenerator::timerCallback()
{
    if (!isScanReceived_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
        return;
    }

    // convert pcl's frame(laser) to robot frame(base_link)
    sensorFrameToRobotFrame(pcl_);

    // convert pcl to costmap
    const std::vector<grid_map::Index> occupiedIndices = pclToCostmap(pcl_, costmap_);

    // publish costmap as grid_map_msgs::msg::GridMap
    std::shared_ptr<grid_map_msgs::msg::GridMap> costmapMsg;
    costmapMsg = grid_map::GridMapRosConverter::toMessage(*costmap_);
    costmapPublisher_->publish(*costmapMsg);

    // publish costmap as nav_msgs::msg::OccupancyGrid
    nav_msgs::msg::OccupancyGrid occupancyGridMsg;
    grid_map::GridMapRosConverter::toOccupancyGrid(*costmap_, "collision_layer", 0.0, 1.0, occupancyGridMsg);
    occupancyGridMsg.header.stamp = this->get_clock()->now();
    occupancyGridMsg.header.frame_id = robotFrameId_;
    costmapPublisher2_->publish(occupancyGridMsg);
}
/**
 * @brief pcl을 받아, 그 좌표의 기준을 laser frame에서 base frame으로 변환한다. 
    따라서 이것은 반드시 '/tf_static'이 laser frame과 base frame 사이의 변환을 포함해야 동작한다.
 * @param pcl pcl::PointXYZ 타입의 포인터
 */
void LocalCostmapGenerator::sensorFrameToRobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl)
{   

    // base_link frame와 laser frame 사이의 변환을 tf를 통해 받는다.
    try {
        transformStamped_ = tf_buffer_.lookupTransform(robotFrameId_, sensorFrameId_, rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[LocalCostmapGenerator] %s", ex.what());
        return;
    }

    // tf를 Eigen의 Isometry3d로 변환한다. Isometry3d는 Transformation Matrix를 나타내는 Eigen의 클래스이다.
    // 3차원의 변환행렬을 나타내므로 tf를 Eigen으로 변환하면 3차원 변환행렬(이는 4x3 행렬이다.)을 얻을 수 있다.
    // 아니 왜 4x3 행렬이지? - 3x3행렬로 나타낼 수 있는 변환을 생각해보자. 원점을 평행이동하는 게 가능한가?
    // 절대 아닐 것이다. 3x3 행렬에 0벡터를 곱하면 0벡터가 나오기 때문이다.
    // 여기에서 트릭을 사용한다. 벡터에 차원을 하나 더하는 것이다. 그리고 그 더해진 차원을 선형변환하면 평행이동이 가능하다.
    // eg. (1;2)벡터가 있다. 이것을 (1 0 ; 0 1) 행렬로 선형변환하면 (1;2)가 나온다. Identity 행렬이기 때문이다.
    // 이것을 (1;2;1)로 차원을 하나 더해 바꾸고, (1 0 1; 0 1 1)로 선형변환하면 (2;3) 벡터가 나온다.
    // 같은 연산을 (0;0)벡터에 대해 해보자.(0;0;1)에 (1 0 1; 0 1 1)을 곱하면 (1;1)이 나온다.
    // 원래 2차원에서 (0;0)인 벡터가 (1;1)로 변환되었다. 이렇게 차원을 하나 더해놓고 선형변환을 하면 평행이동이 가능해진다.
    // 이것이 3차원에서 평행이동을 나타낼 때 4x3 행렬이 나오는 이유이며, 
    // 여기에서 좌표변환을 할 때 eigen을 사용해 변환 행렬을 얻고 행렬곱을 하여 좌표를 변환하는 것이다.
    const Eigen::Isometry3d transformMatrix = tf2::transformToEigen(transformStamped_.transform);

    // laser frame 기준 pcl(좌표)을 변환행렬을 통해 base_link frame의 pcl(좌표)로 변환한다.
    pcl::transformPointCloud(*pcl, *pcl, transformMatrix.matrix().cast<float>());
}

/**
 * @brief pcl을 받아, 그 좌표를 costmap의 좌표로 변환한다.
 * @param pcl pcl::PointXYZ 타입의 포인터
 * @param costmap grid_map::GridMap 타입의 포인터
 * @return std::vector<grid_map::Index> 변환된 좌표의 인덱스
 */
std::vector<grid_map::Index> LocalCostmapGenerator::pclToCostmap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl, 
    grid_map::GridMap* costmap
) const
{
    // transform pcl to costmap
    std::vector<grid_map::Index> occupiedIndices(pcl->points.size(), grid_map::Index(0, 0));

    costmap->get("collision_layer").setConstant(0.0); // costmap 초기화
    for (unsigned int index = 0; index < pcl->points.size(); index++) {
        const auto& point = pcl->points[index];

        const double len = gridLength / 2; // grid map의 길이(m)의 절반

        if (point.x >= -len && point.x <= len && point.y >= -len && point.y <= len) {
            int grid_x = static_cast<int>((point.x + len) / resolution); // x 좌표를 그리드 인덱스로 변환
            int grid_y = static_cast<int>((point.y + len) / resolution); // y 좌표를 그리드 인덱스로 변환

            // 그리드 인덱스가 범위를 벗어나지 않도록 체크
            if (grid_x >= 0 && grid_x < gridSize && grid_y >= 0 && grid_y < gridSize) {
                occupiedIndices[index] = grid_map::Index(grid_x, grid_y);
                // update costmap
                costmap->atPosition("collision_layer", grid_map::Position(point.x, point.y)) = 1.0;
            } else {
                occupiedIndices[index] = grid_map::Index(-1, -1); // 범위를 벗어난 인덱스
            }
        } else {
            occupiedIndices[index] = grid_map::Index(-1, -1);
        }
    }

    // // 디버깅용 출력: 행렬 시각화. costmap의 데이터를 출력한다. 보고싶으면 아래 주석처리를 제거하면 된다.
    // Eigen::MatrixXf& costmapData = costmap->get("collision_layer"); // costmap의 데이터를 가져온다.
    // std::cout << "----------------------------------" << std::endl;
    // std::cout << costmapData << std::endl; // 디버깅용 출력: costmap 시각화
    // std::cout << "----------------------------------" << std::endl;
    
    return occupiedIndices;
}