#include "svg_mppi_planner/svg_mppi_planner_ros.hpp"

namespace svg_mppi { 

SVGMPPIPlannerROS::SVGMPPIPlannerROS() : Node("svg_mppi_planner_node")
{
    svg_mppi_pointer_ = std::make_unique<svg_mppi::planning::SVGMPPI>();

    // initialize local cost map
    local_cost_map_ = new grid_map::GridMap({"collision_layer"});
    local_cost_map_->setGeometry(grid_map::Length(gridLength, gridLength), resolution, grid_map::Position(0.0, 0.0));
    local_cost_map_->get("collision_layer").setConstant(0.0);

    cost_map_subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "cost_map",
        10, 
        std::bind(&SVGMPPIPlannerROS::local_cost_map_callback, this, std::placeholders::_1)
    );

    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "ego_racecar/odom",
        10,
        std::bind(&SVGMPPIPlannerROS::odometry_callback, this, std::placeholders::_1)
    );

    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "candidate_paths",
        10
    );

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "candidate_path",
        10
    );

    path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "path",
        10
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period), std::bind(&SVGMPPIPlannerROS::timer_callback, this)
    );
}

void SVGMPPIPlannerROS::timer_callback()
{
    // check status
    if (!is_local_cost_map_received_ || !is_odometry_received_) {
        std::cout << "[SVGMPPIPlannerROS] Not ready: odometry: " << is_odometry_received_
                  << ", local cost map: " << is_local_cost_map_received_ << std::endl;

        return;
    }

    // set local cost map for planning
    svg_mppi_pointer_->set_local_cost_map(
        *local_cost_map_
    );

    // steer observer

    // svg mppi solve
    svg_mppi::planning::State initial_state = svg_mppi::planning::State::Zero();
    initial_state[STATE_SPACE::x] = robot_state_.x;
    initial_state[STATE_SPACE::y] = robot_state_.y;
    initial_state[STATE_SPACE::yaw] = robot_state_.yaw;
    initial_state[STATE_SPACE::velocity] = robot_state_.velocity;
    initial_state[STATE_SPACE::steering] = robot_state_.steering;

    const auto [updated_control_sequence, updated_collision_rate] = svg_mppi_pointer_->solve(initial_state);

    // visualize_state_sequence(
    //     svg_mppi_pointer_->state_sequence_batch_[0],
    //     "state_sequence",
    //     "r"
    // );

    // std::vector<double> weight_batch = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    // visualize_state_sequence_batch(
    //     svg_mppi_pointer_->state_sequence_batch_,
    //     weight_batch
    // );


    // // debeg
    // planning::StateSequenceBatch state_sequence_batch = {
    //     Eigen::MatrixXd::Random(10, 5),
    //     Eigen::MatrixXd::Random(10, 5),
    //     Eigen::MatrixXd::Random(10, 5)
    // };

    // std::vector<double> weight_batch = {1.0, 2.0, 3.0};
    // visualize_state_sequence_batch(
    //     state_sequence_batch,
    //     weight_batch
    // );


    // // test code for visualize_state_sequence function
    // // please remove this code after implementing the function
    // planning::StateSequence state_sequence = Eigen::MatrixXd::Random(10, 5);

    // visualize_state_sequence(
    //     state_sequence,
    //     "state_sequence",
    //     "r"
    // );

    // // test code for visualize_path function
    // // please remove this code after implementing the function
    // planning::StateSequence path = Eigen::MatrixXd::Random(10, 5);

    // visualize_path(
    //     path,
    //     "path",
    //     "g"
    // );
}

void SVGMPPIPlannerROS::local_cost_map_callback(
    const grid_map_msgs::msg::GridMap::SharedPtr local_cost_map
)
{
    // Convert grid_map_msgs::msg::GridMap 2 grid_map::GridMap
    if (!grid_map::GridMapRosConverter::fromMessage(*local_cost_map, *local_cost_map_)) {
        RCLCPP_ERROR(this->get_logger(), "[MPPIPlannerROS] Failed to convert grid_map_msgs::msg::GridMap to grid_map::GridMap");
        return;
    }

    // flag for if received local cost map
    is_local_cost_map_received_ = true;

    // for debugging
    // Eigen::MatrixXf& costmapData = local_cost_map_->get("collision_layer");
    // std::cout << "----------------------------------" << std::endl;
    // std::cout << costmapData << std::endl;
    // std::cout << "----------------------------------" << std::endl;
}

void SVGMPPIPlannerROS::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr odometry
)
{
    // localizeless mode
    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.yaw = 0.0;
    robot_state_.velocity = odometry->twist.twist.linear.x;
    
    is_odometry_received_ = true;

    // // for debug
    // // print position
    // std::cout << "Position:"
    //           << "\n  x: " << odometry->pose.pose.position.x
    //           << "\n  y: " << odometry->pose.pose.position.y
    //           << "\n  z: " << odometry->pose.pose.position.z 
    //           << std::endl;

    // // print velocity
    // std::cout << "Velocity:"
    //           << "\n  linear x: " << odometry->twist.twist.linear.x
    //           << "\n  linear y: " << odometry->twist.twist.linear.y
    //           << "\n  linear z: " << odometry->twist.twist.linear.z
    //           << "\n  angular x: " << odometry->twist.twist.angular.x
    //           << "\n  angular y: " << odometry->twist.twist.angular.y
    //           << "\n  angular z: " << odometry->twist.twist.angular.z 
    //           << std::endl;
}

void SVGMPPIPlannerROS::visualize_state_sequence_batch(
    const planning::StateSequenceBatch& state_sequence_batch,
    const std::vector<double>& weight_batch
){
    assert(state_sequence_batch.size() == weight_batch.size());

    visualization_msgs::msg::MarkerArray marker_array;

    const double max_weight = weight_batch[0];
    const double max_node_size = 0.05;
    for (size_t i = 0; i < state_sequence_batch.size(); i++) {
        visualization_msgs::msg::Marker line;

        line.header.frame_id = robot_frame_id_;
        line.header.stamp = this->get_clock()->now();
        line.ns = "candidate_path_line";
        line.id = i;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.x = 0.0;
        line.pose.orientation.y = 0.0;
        line.pose.orientation.z = 0.0;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.01;
        line.color.a = 0.3;
        line.color.r = 0.0;
        line.color.g = 0.5;
        line.color.b = 1.0;
        // line.lifetime = ros::Duration(0.1);

        // nodes
        visualization_msgs::msg::Marker nodes;

        nodes.header.frame_id = robot_frame_id_;
        nodes.header.stamp = this->get_clock()->now();
        nodes.ns = "candidate_path_nodes";
        nodes.id = line.id + 10000 + i;
        nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes.action = visualization_msgs::msg::Marker::ADD;
        nodes.pose.orientation.x = 0.0;
        nodes.pose.orientation.y = 0.0;
        nodes.pose.orientation.z = 0.0;
        nodes.pose.orientation.w = 1.0;
        nodes.scale.x = weight_batch[i] * max_node_size / max_weight + 0.01;
        nodes.scale.y = 0.01;
        nodes.scale.z = 0.01;
        nodes.color.a = 0.6;
        nodes.color.r = 0.0;
        nodes.color.g = 0.5;
        nodes.color.b = 1.0;
        // nodes.lifetime = ros::Duration(0.1);

        for (int j = 0; j < state_sequence_batch.at(0).rows(); j++) {
            geometry_msgs::msg::Point point;
            point.x = state_sequence_batch.at(i)(j, STATE_SPACE::x);
            point.y = state_sequence_batch.at(i)(j, STATE_SPACE::y);
            point.z = 0.0;
            line.points.push_back(point);
            nodes.points.push_back(point);
        }
        marker_array.markers.push_back(line);
        marker_array.markers.push_back(nodes);
    }

    marker_array_publisher_->publish(marker_array);

}

void SVGMPPIPlannerROS::visualize_state_sequence(
    const planning::StateSequence& state_sequence,
    const std::string& name_space,
    const std::string& rgb
){
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker arrow;

    arrow.header.frame_id = robot_frame_id_;
    arrow.header.stamp = this->get_clock()->now();
    arrow.ns = name_space;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Vector3 arrow_scale;
    arrow_scale.x = 0.02;
    arrow_scale.y = 0.04;
    arrow_scale.z = 0.1;
    arrow.scale = arrow_scale;
    arrow.pose.position.x = 0.0;
    arrow.pose.position.y = 0.0;
    arrow.pose.position.z = 0.0;
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = 0.0;
    arrow.pose.orientation.z = 0.0;
    arrow.pose.orientation.w = 1.0;
    arrow.color.a = 1.0;
    arrow.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
    arrow.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
    arrow.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);

    // arrow.lifetime = ros::Duration(0.1);
    arrow.points.resize(2);

    for (int i = 0; i < state_sequence.rows(); i++) {
        arrow.id = i;
        const auto state = state_sequence.row(i);
        const double length = abs(state[STATE_SPACE::velocity]) * 0.1;
        geometry_msgs::msg::Point start;
        start.x = state[STATE_SPACE::x];
        start.y = state[STATE_SPACE::y];
        start.z = 0.1;

        geometry_msgs::msg::Point end;
        end.x = state[STATE_SPACE::x] + length * cos(state[STATE_SPACE::yaw]);
        end.y = state[STATE_SPACE::y] + length * sin(state[STATE_SPACE::yaw]);
        end.z = 0.1;

        arrow.points[0] = start;
        arrow.points[1] = end;

        marker_array.markers.push_back(arrow);
    }
    marker_publisher_->publish(marker_array);
}

void SVGMPPIPlannerROS::visualize_path(
    const planning::StateSequence& state_sequence,
    const std::string& name_space,
    const std::string& rgb
)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker line;

    line.header.frame_id = robot_frame_id_;
    line.header.stamp = this->get_clock()->now();
    line.ns = name_space + "_line";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.01;
    line.color.a = 1.0;
    line.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
    line.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
    line.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);
    // line.lifetime = ros::Duration(0.1);

    // nodes
    visualization_msgs::msg::Marker nodes;

    nodes.header.frame_id = robot_frame_id_;
    nodes.header.stamp = this->get_clock()->now();
    nodes.ns = name_space + "_nodes";
    nodes.id = 1;
    nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodes.action = visualization_msgs::msg::Marker::ADD;
    nodes.pose.orientation.x = 0.0;
    nodes.pose.orientation.y = 0.0;
    nodes.pose.orientation.z = 0.0;
    nodes.pose.orientation.w = 1.0;
    const double node_size = 0.1;
    nodes.scale.x = node_size;
    nodes.scale.y = node_size;
    nodes.scale.z = node_size;
    nodes.color.a = 1.0;
    nodes.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
    nodes.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
    nodes.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);
    // nodes.lifetime = ros::Duration(0.1);

    for (int j = 0; j < state_sequence.rows(); j++) {
        geometry_msgs::msg::Point point;
        point.x = state_sequence(j, STATE_SPACE::x);
        point.y = state_sequence(j, STATE_SPACE::y);
        point.z = 0.1;
        line.points.push_back(point);
        nodes.points.push_back(point);
    }
    marker_array.markers.push_back(line);
    marker_array.markers.push_back(nodes);

    path_publisher_->publish(marker_array);
}

} // namespace svg_mppi