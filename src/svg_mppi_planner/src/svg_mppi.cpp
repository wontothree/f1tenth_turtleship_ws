#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {
namespace planning {

SVGMPPI::SVGMPPI() 
{
    test();
}


std::pair<ControlMeanTrajectory, double> SVGMPPI::solve(const State& current_state)
{
    // transport guide particles by stein variational gradient descent
    std::vector<double> cost_history;
    std::vector<ControlMeanTrajectory> control_trajectory_history;

    for (int i = 0; i < num_svgd_iteration_; i++) {
        // transport samples by stein variational gradient descent
        // const ControlMeanTrajectoryBatch gradient_log_posterior_;
    }
}


std::pair<std::vector<double>, std::vector<double>> SVGMPPI::calculate_sample_costs(
    const State& current_state,
    const grid_map::GridMap& local_cost_map,
    StateMeanTrajectoryBatch* state_trajectory_candidates
) const
{
    std::vector<double> cost_(sample_number_);
    std::vector<double> collision_costs_(sample_number_);

    for (size_t i = 0; i < sample_number_; i++) {
        // state_trajectory_candidates->at(i); // predict_state_trajectory

        // calculate cost
        // const auto [cost_, collision_cost_]; // calculate_state_cost

        // cost_.at(i) = cost_;
        // collision_cost_.at(i) = collision_cost_;
    }

    // return std::make_pair(cost_, collision_cost_);
}

StateMeanTrajectory SVGMPPI::predict_state_trajectory(
    const ControlMeanTrajectory& control_trajectory,
    const State& current_state,
    const grid_map::GridMap& local_cost_map
) const
{
    // initializer state trajectory as 0
    StateMeanTrajectory state_trajectory_ = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);

    // set current state to state trajectory as initial state
    state_trajectory_.row(0) = current_state;

    for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
        // steering 지연 무시 -> 나중에 고려해야 함
        double steering_angle_ = 0.0;
        steering_angle_ = control_trajectory(i, CONTROL_SPACE::steering);

        const double x_ = state_trajectory_(i, STATE_SPACE::x);
        const double y_ = state_trajectory_(i, STATE_SPACE::y);;
        const double yaw_ = state_trajectory_(i, STATE_SPACE::yaw);
        const double velocity_ = state_trajectory_(i, STATE_SPACE::velocity);
        const double steering_ = state_trajectory_(i, STATE_SPACE::steering);

        // kinematic bicycle model
        const double sideslip_ = atan(lf_ / (lf_ + lr_) * tan(steering_angle_));
        const double delta_x_ = velocity_ * cos(yaw_ + sideslip_) * prediction_interval_;
        const double delta_y_ = velocity_ * sin(yaw_ + sideslip_) * prediction_interval_;
        const double delta_yaw_ = velocity_ * sin(sideslip_) / lr_ * prediction_interval_;
        const double delta_steering_ = steering_;

        double next_velocity_ = 0.0;
        if (1) {
            next_velocity_ = predict_constant_speed(velocity_);
        }

        // next state
        state_trajectory_(i + 1, STATE_SPACE::x) = x_ + delta_x_;
        state_trajectory_(i + 1, STATE_SPACE::y) = y_ + delta_y_;
        state_trajectory_(i + 1, STATE_SPACE::yaw) = std::atan2(sin(yaw_ + delta_yaw_), cos(yaw_ + delta_yaw_));
        state_trajectory_(i + 1, STATE_SPACE::velocity) = next_velocity_;
        state_trajectory_(i + 1, STATE_SPACE::steering) = steering_ + delta_steering_;
    }

    return state_trajectory_;
}

double SVGMPPI::predict_constant_speed(
    const double& current_speed
) const
{
    return current_speed;
}

// 하나의 state trajectory에 대한 하나의 비용을 계산한다.
std::pair<double, double> SVGMPPI::calculate_state_cost(
    const StateMeanTrajectory state_trajectory,
    const grid_map::GridMap& local_cost_map
) const
{
    // total cost of summing state cost and terminal cost
    double collision_cost_sum_ = 0.0;

    // state cost
    for (size_t i = 0; i < prediction_step_size_; i++) {
        double collision_state_cost_ = 0.0;

        State state_ = state_trajectory.row(i);
        if (local_cost_map.isInside(grid_map::Position(state_(STATE_SPACE::x), state_(STATE_SPACE::y)))) {
            collision_state_cost_ = local_cost_map.atPosition("collision", grid_map::Position(state_(STATE_SPACE::x), state_(STATE_SPACE::y)));
        }

        collision_cost_sum_ += collision_weight_ * collision_state_cost_;
    }

    // terminal cost
    const State terminal_state_ = state_trajectory.row(prediction_step_size_ - 1);
    double collision_terminal_cost_ = 0.0;
    if (local_cost_map.isInside(grid_map::Position(terminal_state_(STATE_SPACE::x), terminal_state_(STATE_SPACE::y)))) {
        collision_terminal_cost_ = local_cost_map.atPosition("collision", grid_map::Position(terminal_state_(STATE_SPACE::x), terminal_state_(STATE_SPACE::y)));
    }

    collision_cost_sum_ += collision_weight_ * collision_terminal_cost_;

    return std::make_pair(collision_cost_sum_, collision_cost_sum_);
}

ControlMeanTrajectoryBatch SVGMPPI::approximate_gradient_posterior_batch() const
{
    ControlMeanTrajectoryBatch gradient_log_posterior_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        10, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim) // 10은 임시
    );



}

void SVGMPPI::test()
{   
    // test function predict_constant_speed
    double tmp = predict_constant_speed(2.0);
    // std::cout << tmp << std::endl;

    // -------------------------------------------------------------------------------------------------------------------------------

    // test function predict_state_trajectory
    State current_state;
    current_state << 0.0, 0.0, 0.0, 1.0, 0.1;
    ControlMeanTrajectory control_trajectory = Eigen::MatrixXd::Zero(prediction_step_size_, CONTROL_SPACE::dim);
    // for (size_t i = 0; i < prediction_step_size_; i++) {
    //     control_trajectory(i, STATE_SPACE::steering) = 0.1;
    // }
    grid_map::GridMap local_cost_map;

    StateMeanTrajectory predicted_trajectory = predict_state_trajectory(control_trajectory, current_state, local_cost_map);

    // std::cout << "Predicted State Trajectory:\n";
    // std::cout << predicted_trajectory << std::endl;

    // -------------------------------------------------------------------------------------------------------------------------------

    // test function calculate_state_cost


} 

} // namespace planning
} // namespace svg_mppi
