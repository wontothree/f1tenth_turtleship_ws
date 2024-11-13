
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