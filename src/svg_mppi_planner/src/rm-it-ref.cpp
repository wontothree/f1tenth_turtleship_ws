
void MPPIControllerROS::publish_traj(const mppi::cpu::StateSeq& state_seq,
                                     const std::string& name_space,
                                     const std::string& rgb,
                                     const ros::Publisher& publisher) const {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker arrow;
    if (is_localize_less_mode_) {
        arrow.header.frame_id = robot_frame_id_;
    } else {
        arrow.header.frame_id = map_frame_id_;
    }
    arrow.header.stamp = ros::Time::now();
    arrow.ns = name_space;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 arrow_scale;
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

    for (int i = 0; i < state_seq.rows(); i++) {
        arrow.id = i;
        const auto state = state_seq.row(i);
        const double length = abs(state[STATE_SPACE::vel]) * 0.1;
        geometry_msgs::Point start;
        start.x = state[STATE_SPACE::x];
        start.y = state[STATE_SPACE::y];
        start.z = 0.1;

        geometry_msgs::Point end;
        end.x = state[STATE_SPACE::x] + length * cos(state[STATE_SPACE::yaw]);
        end.y = state[STATE_SPACE::y] + length * sin(state[STATE_SPACE::yaw]);
        end.z = 0.1;

        arrow.points[0] = start;
        arrow.points[1] = end;

        marker_array.markers.push_back(arrow);
    }
    publisher.publish(marker_array);
}

void MPPIControllerROS::publish_path(const mppi::cpu::StateSeq& state_seq,
                                     const std::string& name_space,
                                     const std::string& rgb,
                                     const ros::Publisher& publisher) const {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker line;
    if (is_localize_less_mode_) {
        line.header.frame_id = robot_frame_id_;
    } else {
        line.header.frame_id = map_frame_id_;
    }
    line.header.stamp = ros::Time::now();
    line.ns = name_space + "_line";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
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
    visualization_msgs::Marker nodes;
    if (is_localize_less_mode_) {
        nodes.header.frame_id = robot_frame_id_;
    } else {
        nodes.header.frame_id = map_frame_id_;
    }
    nodes.header.stamp = ros::Time::now();
    nodes.ns = name_space + "_nodes";
    nodes.id = 1;
    nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes.action = visualization_msgs::Marker::ADD;
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

    for (int j = 0; j < state_seq.rows(); j++) {
        geometry_msgs::Point point;
        point.x = state_seq(j, STATE_SPACE::x);
        point.y = state_seq(j, STATE_SPACE::y);
        point.z = 0.1;
        line.points.push_back(point);
        nodes.points.push_back(point);
    }
    marker_array.markers.push_back(line);
    marker_array.markers.push_back(nodes);

    publisher.publish(marker_array);
}