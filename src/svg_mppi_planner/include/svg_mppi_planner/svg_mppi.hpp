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
    // Constant
    const int thread_number_ = 4;
    const size_t sample_number_ = 5;
    const size_t prediction_horizon_ = 4;
    const size_t prediction_step_size_ = 4;
    const double prediction_interval_ = 0.05; // s
    const double lf_ = 0.189;
    const double lr_ = 0.135;
    const double collision_weight_ = 1.0;
    const size_t svgd_iteration_number_ = 3;
    const std::array<double, CONTROL_SPACE::dim> steering_control_covariance_for_gradient_estimation_ = {0.01};
    const double max_steering_ = 0.45;
    const double min_steering_ = -0.45;
    const double non_biased_sampling_rate_ = 0.1;
    const double grad_lambda_ = 3.0;

    // for random_sampling
    ControlSequence control_mean_sequence_;
    ControlCovarianceSequence control_covariance_sequence_;
    ControlSequenceBatch noise_sequence_batch_;
    ControlSequenceBatch noised_control_sequence_batch_;

    // for approximate_gradient_log_likelihood
    StateSequenceBatch state_sequence_batch_;
    ControlCovarianceSequence control_inverse_covariance_sequence_;
    ControlSequence previous_control_mean_sequence_;

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

    std::pair<ControlSequence, double> solve(
        const State& initial_state
    );

    /**
     * @brief
     */
    ControlSequenceBatch approximate_gradient_log_posterior_batch(
        const State& initial_state,
        const ControlSequence control_sequence
    );
    /**
     * @brief
     */
    ControlSequence approximate_gradient_log_likelihood(
        const State& initial_state,
        const ControlSequence& control_mean_sequence,
        const ControlSequence& noised_control_mean_sequence,
        const ControlCovarianceSequence& control_inverse_covariance_sequence
    );

    /**
     * @brief
     */
    void random_sampling(
        const ControlSequence control_mean_sequence,
        const ControlCovarianceSequence control_covariance_sequence
    );

    // --------------------------------------------------------------------------------------------------------------------------------
    // to do 이진우

    /**
     * @brief 모든 샘플에 대한 각각의 state cost를 계산한다.
     * @param initial_state 초기 상태
     */
    std::pair<std::vector<double>, std::vector<double>> calculate_state_cost_batch(
        const State& initial_state,
        const grid_map::GridMap& local_cost_map,
        StateSequenceBatch* state_sequence_batch,
        ControlSequenceBatch& control_sequence_batch
    ) const;

    /**
     * @brief 하나의 state sequence를 예측한다.
     */
    StateSequence predict_state_sequence(
        const State& initial_state,
        const ControlSequence& control_sequence
    ) const;

    double predict_constant_speed(
        const double& current_speed
    ) const
    {
        return current_speed;
    }

    /**
     * @brief 하나의 sequence에 대한 cost를 계산한다.
     */
    std::pair<double, double> calculate_state_sequence_cost(
        const StateSequence state_sequence,
        const grid_map::GridMap& local_cost_map
    ) const;

    // --------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Computes the weights for state trajectories using softmax
     * 
     * Calculates the weight for each state trajectory based on state cost and control input differences.
     * 
     * @param lambda Regularization parameter
     * @param alpha Control input difference regularization
     * @param state_cost Vector of state costs
     * @param initial_control_sequence Initial control sequence
     * @param nominal_control_sequence Nominal control sequence
     * @param control_sequence Batch of control sequences
     * @return A vector of weights for each trajectory
     */
    std::vector<double> calculate_sample_cost_batch(
        const double& lambda,
        const double& alpha,
        const std::vector<double> state_costs,
        const ControlSequence initial_consrol_sequence,
        const ControlSequence nominal_control_sequence,
        const ControlSequenceBatch control_sequence,
        const ControlCovarianceSequence control_inverse_covariance_sequence
    ) const;

    /**
     * @brief Calculate the softmax of the given costs.
     * @param costs A vector of costs to be normalized.
     * @param lambda A scaling parameter for the softmax function.
     * @param thread_number The number of threads to use for parallel computation.
     * @return
     */
    std::vector<double> softmax(
        const std::vector<double>& costs,
        const double& lambda,
        const int thread_number
    ) const
    {
        // minimum element of costs
        const double min_cost_ = *std::min_element(costs.begin(), costs.end());

        // calculate normalizing constant
        double normalizing_constant_ = 1e-10;
        for (const auto& cost : costs) {
            normalizing_constant_ += std::exp(- (cost - min_cost_) / lambda);
        }

        // calculate softmax of costs
        std::vector<double> softmax_costs_(costs.size());
        #pragma omp parallel for num_threads(thread_number)
        for (size_t i = 0; i < costs.size(); i++) {
            softmax_costs_[i] = std::exp(-(costs[i] - min_cost_) / lambda) / normalizing_constant_;
        }

        return softmax_costs_;
    }

    void set_local_cost_map(
        const grid_map::GridMap& local_cost_map
    )
    {
        local_cost_map_ = local_cost_map;
    }

    void set_control_mean_sequence(
        const ControlSequence& control_mean_sequence
    )
    {
        control_mean_sequence_ = control_mean_sequence;
    }

    void set_control_covariance_sequence(
        const ControlCovarianceSequence& control_covariance_sequence
    )
    {
        control_covariance_sequence_ = control_covariance_sequence;

        // small value to add diagonal elements for preventing singular matrix
        const double epsilon_ = 1e-4;


        // calculate inverse of covariance matrices in advance to reduce computational cost
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            control_inverse_covariance_sequence_[i] = control_covariance_sequence_[i].inverse() + epsilon_ * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
        }
    }

};

} // namespace planning
} // namespace svg_mppi