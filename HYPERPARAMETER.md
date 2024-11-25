# Hyperparameter

This page is for all tuning parameter in f1tenth vehicle

# Vesc

# Global Planner

# MPPI Controller

|Index|Parameter Name|Setting|default|Function|
|---|---|---|---|---|
|1|control_cmd_topic|vesc/high_level/ackermann_cmd_mux/input/nav_1||
|2|in_reference_sdf_topic|reference_sdf|||
|3|in_odom_topic|car_state/odom|||
|4|is_activate_ad_topic|is_active_ad|||
|5|robot_frame_id|base_link|||
|6|map_frame_id|map|||
|7|costmap_id|f1_costmap_2d/f1_costmap/costmap|||
|8|local_costmap_id|local_costmap|||
|9|backward_point_topic|backward_point|||
|10|control_sampling_time|0.025|||
||||||
||thread_num|12|12|
||prediction_step_size|20|15||
||prediction_interval|0.04|0.05||
||steer_delay|0.01|0.01||
||max_steer_angle|0.45|0.45||
||min_steer_angle|-0.45|-0.45||
||speed_prediction_mode|reference|reference||
||max_accel|5.0|5.0||
||min_accel|-3.0|-3.0||
||lr|0.135|0.135||
||lf|0.189|0.189||
||collision_weight|1.0|1.0||
||q_dist||||
||q_angle||||
|
