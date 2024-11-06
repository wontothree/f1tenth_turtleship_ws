#pragma once

#include "rclcpp/rclcpp.hpp"
#include <random>
#include <omp.h>

#include <grid_map_core/GridMap.hpp>

#include "svg_mppi_planner/common.hpp"

namespace svg_mppi {
namespace planning {

class SVGMPPI {
private:
    const size_t sample_number_ = 10;
    const size_t prediction_step_size_ = 3;
    const size_t prediction_horizon_ = 3;
    const double prediction_interval_ = 0.05; // s
    const double lf_ = 0.189;
    const double lr_ = 0.135;
    const double collision_weight_ = 1.0;
    const size_t num_svgd_iteration_ = 3;
    const size_t thread_number_ = 4;
    const double non_biased_sampling_rate_ = 0.1;

    ControlMeanTrajectory control_mean_trajectory_;
    ControlCovarianceTrajectory control_covariance_trajectory_;
    ControlCovarianceTrajectory control_inverse_covariance_trajectory_;

    ControlMeanTrajectoryBatch noise_sample_trajectory_batch_;
    ControlMeanTrajectoryBatch noised_control_mean_sample_trajectory_batch_;

    // generate random noise
    std::vector<std::mt19937> random_number_generators_;
    std::unique_ptr<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>> normal_distribution_pointer_;

public:
SVGMPPI();
~SVGMPPI() {};

std::pair<ControlMeanTrajectory, double> solve(const State& current_state);

std::pair<std::vector<double>, std::vector<double>> calculate_sample_costs(
    const State& current_state,
    const grid_map::GridMap& local_cost_map,
    StateMeanTrajectoryBatch* state_trajectory_candidates
) const;

StateMeanTrajectory predict_state_trajectory(
    const ControlMeanTrajectory& control_trajectory,
    const State& current_state,
    const grid_map::GridMap& local_cost_map
) const;

double predict_constant_speed(
    const double& current_speed
) const;

std::pair<double, double> calculate_state_cost(
    const StateMeanTrajectory state_trajectory,
    const grid_map::GridMap& local_cost_map
) const;

// for stein variational gradient descent
ControlMeanTrajectoryBatch approximate_gradient_posterior_batch() const;
ControlMeanTrajectory approximate_gradient_log_likelihood() const;

void random_sampling(
    const ControlMeanTrajectory& control_mean_trajectory,
    const ControlCovarianceTrajectory& control_covariance_trajectory
);

void test();

void set_control_mean_trajectory(
    const ControlMeanTrajectory& control_mean_trajectory
)
{
    control_mean_trajectory_ = control_mean_trajectory;
}
void set_control_covariance_trajectory(
    const ControlCovarianceTrajectory& control_covariance_trajectory
)
{
    control_covariance_trajectory_ = control_covariance_trajectory;

    // small value to add diagonal elements for preventing singular matrix
    const double epsilon_ = 1e-4;

    // calculate inverse of covariance matrices in advance to reduce computational cost
    for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
        control_inverse_covariance_trajectory_[i] = control_covariance_trajectory_[i].inverse() + epsilon_ * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
    }
}

};

} // namespace planning
} // namespace svg_mppi