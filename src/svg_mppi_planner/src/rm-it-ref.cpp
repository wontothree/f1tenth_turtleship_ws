void MPPIControllerROS::publish_candidate_paths(const std::vector<mppi::cpu::StateSeq>& state_seq_batch,
                                                const std::vector<double>& weights,
                                                const ros::Publisher& publisher) const {
    assert(state_seq_batch.size() == weights.size());

    visualization_msgs::MarkerArray marker_array;

    const double max_weight = weights[0];
    const double max_node_size = 0.05;
    for (size_t i = 0; i < state_seq_batch.size(); i++) {
        visualization_msgs::Marker line;
        if (is_localize_less_mode_) {
            line.header.frame_id = robot_frame_id_;
        } else {
            line.header.frame_id = map_frame_id_;
        }
        line.header.stamp = ros::Time::now();
        line.ns = "candidate_path_line";
        line.id = i;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
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
        visualization_msgs::Marker nodes;
        if (is_localize_less_mode_) {
            nodes.header.frame_id = robot_frame_id_;
        } else {
            nodes.header.frame_id = map_frame_id_;
        }
        nodes.header.stamp = ros::Time::now();
        nodes.ns = "candidate_path_nodes";
        nodes.id = line.id + 10000 + i;
        nodes.type = visualization_msgs::Marker::SPHERE_LIST;
        nodes.action = visualization_msgs::Marker::ADD;
        nodes.pose.orientation.x = 0.0;
        nodes.pose.orientation.y = 0.0;
        nodes.pose.orientation.z = 0.0;
        nodes.pose.orientation.w = 1.0;
        nodes.scale.x = weights[i] * max_node_size / max_weight + 0.01;
        nodes.scale.y = 0.01;
        nodes.scale.z = 0.01;
        nodes.color.a = 0.6;
        nodes.color.r = 0.0;
        nodes.color.g = 0.5;
        nodes.color.b = 1.0;
        // nodes.lifetime = ros::Duration(0.1);

        for (int j = 0; j < state_seq_batch.at(0).rows(); j++) {
            geometry_msgs::Point point;
            point.x = state_seq_batch.at(i)(j, STATE_SPACE::x);
            point.y = state_seq_batch.at(i)(j, STATE_SPACE::y);
            point.z = 0.0;
            line.points.push_back(point);
            nodes.points.push_back(point);
        }
        marker_array.markers.push_back(line);
        marker_array.markers.push_back(nodes);
    }
    publisher.publish(marker_array);
}