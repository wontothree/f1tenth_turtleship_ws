#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {
namespace planning {

SVGMPPI::SVGMPPI() 
{
    //
}

std::pair<ControlMeanTrajectory, double> SVGMPPI::solve(const State& current_state)
{
    // transport guide particles by stein variational gradient descent
    std::vector<double> cost_history;
    std::vector<ControlMeanTrajectory> control_trajectory_history;
    // auto costs;
}


std::pair<std::vector<double>, std::vector<double>> SVGMPPI::calculate_sample_cost(
    const State& current_state,
    const grid_map::GridMap& local_cost_map,
    StateTrajectoryBatch* state_trajectory_candidates
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
        // steering 지연 고려

        const double x = state_trajectory_(i, STATE_SPACE::x);
        const double y = state_trajectory_(i, STATE_SPACE::y);;
        const double yaw = state_trajectory_(i, STATE_SPACE::yaw);
        const double velocity = state_trajectory_(i, STATE_SPACE::velocity);
        const double steering = state_trajectory_(i, STATE_SPACE::steering);

        // kinematic bicycle model
        const double sideslip = atan(lf_ / (lf_ + lr_) * tan(steering));
        const double delta_x = velocity * cos(yaw + sideslip) * prediction_interval_;
        const double delta_y = velocity * sin(yaw + sideslip) * prediction_interval_;
        const double delta_yaw = velocity * sin(sideslip) / lr_ * prediction_interval_;
        const double delta_steering = 0; // check

        double nex_velocity_ = 0.0;

        // next state
        state_trajectory_(i + 1, STATE_SPACE::x) = x + delta_x;
        state_trajectory_(i + 1, STATE_SPACE::y) = y + delta_y;
        state_trajectory_(i + 1, STATE_SPACE::yaw) = std::atan2(sin(yaw + delta_yaw), cos(yaw + delta_yaw));
        state_trajectory_(i + 1, STATE_SPACE::velocity) = nex_velocity_;
        state_trajectory_(i + 1, STATE_SPACE::steering) = steering + delta_steering;
    }

    return state_trajectory_;
}

} // namespace planning
} // namespace svg_mppi