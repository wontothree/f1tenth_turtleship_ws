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
    const size_t sample_number_ = 5;
    const size_t prediction_step_size_ = 4;
    const size_t prediction_horizon_ = 4;
    const double prediction_interval_ = 0.05; // s
    const double lf_ = 0.189;
    const double lr_ = 0.135;
    const double collision_weight_ = 1.0;
    const size_t num_svgd_iteration_ = 3;
    const size_t thread_number_ = 4;
    const double non_biased_sampling_rate_ = 0.1;
    const double max_steering_ = 0.45;
    const double min_steering_ = -0.45;
    const std::array<double, CONTROL_SPACE::dim> steering_control_covariance_for_gradient_estimation_ = {0.01};
    const double grad_lambda_ = 3.0;
    const double svgd_step_size_ = 0.005;
    const double gaussian_fitting_lambda_ = 0.1;
    const double min_steering_covariance_ = 0.001;
    const double max_steering_covariance_ = 0.1;

    ControlMeanTrajectory control_mean_trajectory_;
    ControlCovarianceTrajectory control_covariance_trajectory_;
    ControlCovarianceTrajectory control_inverse_covariance_trajectory_;

    ControlMeanTrajectoryBatch noise_sample_trajectory_batch_;
    ControlMeanTrajectoryBatch noised_control_mean_sample_trajectory_batch_;

    StateMeanTrajectoryBatch state_trajectory_batch_;

    ControlMeanTrajectory previous_control_mean_trajectory_;

    // generate random noise
    std::vector<std::mt19937> random_number_generators_;
    std::unique_ptr<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>> normal_distribution_pointer_;
    const std::array<double, CONTROL_SPACE::dim> max_control_ = {max_steering_};
    const std::array<double, CONTROL_SPACE::dim> min_control_ = {min_steering_};

    // local cost map
    grid_map::GridMap local_cost_map_;

public:
SVGMPPI();
~SVGMPPI() {};

void set_local_cost_map(
    const grid_map::GridMap& local_cost_map
)
{
    local_cost_map_ = local_cost_map;
}

std::pair<ControlMeanTrajectory, double> solve(const State& current_state);

std::pair<std::vector<double>, std::vector<double>> calculate_sample_cost_batch(
    const State& current_state,
    const grid_map::GridMap& local_cost_map,
    StateMeanTrajectoryBatch* state_trajectory_candidates
) const;

/**
 * 하나의 state trajectory를 생성한다.
 */
StateMeanTrajectory predict_state_trajectory(
    const State& current_state,
    const ControlMeanTrajectory& control_trajectory
) const;

double predict_constant_speed(
    const double& current_speed
) const;

/**
 * 
 */
std::pair<double, double> calculate_state_trajectory_cost(
    const StateMeanTrajectory state_trajectory,
    const grid_map::GridMap& local_cost_map
) const;

// for stein variational gradient descent
ControlMeanTrajectoryBatch approximate_gradient_log_posterior_batch(
    const State& current_state
);
ControlMeanTrajectory approximate_gradient_log_likelihood(
    const State& current_state,
    const ControlMeanTrajectory& mean_trajectory,
    const ControlMeanTrajectory& noised_seq,
    const ControlCovarianceTrajectory& inv_covs
);

void random_sampling(
    const ControlMeanTrajectory& control_mean_trajectory,
    const ControlCovarianceTrajectory& control_covariance_trajectory
);

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

std::pair<double, double> gaussian_fitting(
    const std::vector<double>& x,
    const std::vector<double>& y
) const;

void test();

std::vector<double> softmax(
    const std::vector<double>& costs,
    const double& lambda,
    const int thread_number
) const
{
    const double min_cost_ = *std::min_element(costs.begin(), costs.end());
    double normalizing_constant_ = 1e-10;
    for (const auto& cost : costs) {
        normalizing_constant_ += std::exp(- (cost - min_cost_) / lambda);
    }

    std::vector<double> softmax_costs_(costs.size());
    #pragma omp parallel for num_threads(thread_number)
    for (size_t i = 0; i < costs.size(); i++) {
        softmax_costs_[i] = std::exp(-(costs[i] - min_cost_) / lambda) / normalizing_constant_;
    }

    return softmax_costs_;
}

};

} // namespace planning
} // namespace svg_mppi