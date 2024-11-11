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
    const std::array<double, CONTROL_SPACE::dim> steering_control_covariance_for_gradient_estimation_ = {0.01};
    const double max_steering_ = 0.45;
    const double min_steering_ = -0.45;
    const double non_biased_sampling_rate_ = 0.1;
    const double grad_lambda_ = 3.0;
    const double gaussian_fitting_lambda_ = 0.1;
    const double min_steering_covariance_ = 0.001;
    const double max_steering_covariance_ = 0.1;
    const double lambda_ = 3.0;
    const double alpha_ = 0.1;

    // Constant for SVGD
    const size_t guide_sample_number_ = 1;
    const size_t svgd_iteration_number_ = 3;
    const double svgd_step_size_ = 0.005;
    const size_t sample_number_for_gradient_estimation_ = 100;

    // for random_sampling
    std::vector<std::mt19937> random_number_generators_;
    std::unique_ptr<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>> normal_distribution_pointer_;
    const std::array<double, CONTROL_SPACE::dim> max_control_ = {max_steering_};
    const std::array<double, CONTROL_SPACE::dim> min_control_ = {min_steering_};
    ControlSequence control_mean_sequence_;
    ControlCovarianceSequence control_covariance_sequence_;
    ControlSequenceBatch noise_sequence_batch_;
    ControlSequenceBatch noised_control_sequence_batch_; // object

    // calculate_state_cost_batch (변경될 멤버 변수)
    StateSequenceBatch state_sequence_batch_;

    // for approximate_gradient_log_likelihood
    ControlCovarianceSequence control_inverse_covariance_sequence_;
    ControlSequence previous_control_mean_sequence_;


    // local cost map
    grid_map::GridMap local_cost_map_;

    // solve
    std::vector<double> costs_;
    ControlSequence nominal_control_sequence_;

public:
    SVGMPPI();
    ~SVGMPPI() {};

private:
    /**
     * @brief
     * @param initial_state 
     */
    std::pair<ControlSequence, double> solve(
        const State& initial_state
    );

    /**
     * @brief
     */
    ControlSequenceBatch approximate_gradient_log_posterior_batch(
        const State& initial_state
    );
    /**
     * @brief
     * @param initial_state
     * @param control_mean_sequence
     * @param noised_control_mean_sequence
     * @param control_inverse_covariance_sequence
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

    /**
     * @brief 모든 샘플에 대한 각각의 state cost를 계산한다.
     * @param initial_state
     * @param local_cost_map
     * @param state_sequence_batch 상태를 저장할 멤버변수를 입력한다.
     */
    std::pair<std::vector<double>, std::vector<double>> calculate_state_cost_batch(
        const State& initial_state,
        const grid_map::GridMap& local_cost_map,
        StateSequenceBatch* state_sequence_batch
    ) const;

    /**
     * @brief 차량의 kinematics를 이용해서 하나의 inital state과 하나의 control sequence에 대한 하나의 state sequence를 예측한다.
     * @param initial_state
     * @param control_sequence
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
     * @brief 하나의 state sequence에 대한 cost를 계산한다.
     */
    std::pair<double, double> calculate_state_sequence_cost(
        const StateSequence state_sequence,
        const grid_map::GridMap& local_cost_map
    ) const;

    /**
     * @brief
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

    /**
     * @brief Gaussian fitting
     */
    std::pair<double, double> gaussian_fitting(
        const std::vector<double>& x,
        const std::vector<double>& y
    ) const
    {
        assert(x.size() == y.size());

        // Should y is larger than 0 for log function
        std::vector<double> y_hat(y.size(), 0.0);
        std::transform(y.begin(), y.end(), y_hat.begin(), [](double y) { return std::max(y, 1e-10); });

        // const double epsilon = 1e-8;
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < x.size(); i++) {
            const double y_hat_2 = y_hat[i] * y_hat[i];
            const double y_hat_log = std::log(y_hat[i]);

            A(0, 0) += y_hat_2;
            A(0, 1) += y_hat_2 * x[i];
            A(0, 2) += y_hat_2 * x[i] * x[i];

            A(1, 0) += y_hat_2 * x[i];
            A(1, 1) += y_hat_2 * x[i] * x[i];
            A(1, 2) += y_hat_2 * x[i] * x[i] * x[i];

            A(2, 0) += y_hat_2 * x[i] * x[i];
            A(2, 1) += y_hat_2 * x[i] * x[i] * x[i];
            A(2, 2) += y_hat_2 * x[i] * x[i] * x[i] * x[i];

            b(0) += y_hat_2 * y_hat_log;
            b(1) += y_hat_2 * x[i] * y_hat_log;
            b(2) += y_hat_2 * x[i] * x[i] * y_hat_log;
        }

        // solve Au = b
        const Eigen::Vector3d u = A.colPivHouseholderQr().solve(b);

        // calculate mean and variance

        // original
        // const double mean = -u(1) / (2.0 * u(2));
        // const double variance = std::sqrt(-1.0 / (2.0 * u(2)));

        // To avoid nan;
        const double eps = 1e-5;
        const double mean = -u(1) / (2.0 * std::min(u(2), -eps));
        // const double variance = std::sqrt(1.0 / (2.0 * std::abs(std::min(u(2), -eps))));
        const double variance = std::sqrt(1.0 / (2.0 * std::abs(u(2))));

        return std::make_pair(mean, variance);
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

    void test()
    {
        // State state_tmp_;
        // state_tmp_ << 0.0, 0.0, 0.0, 1.0, 0.1;
        // std::cout << "state_tmp_" << std::endl;
        // std::cout << state_tmp_ << std::endl;


        // ControlSequence control_mean_trajectory_tmp_ = Eigen::MatrixXd::Zero(
        //     prediction_horizon_ - 1, CONTROL_SPACE::dim
        // );
        // control_mean_trajectory_tmp_ << 0.1, 0.2, 0.3;
        // std::cout << "control_mean_trajectory_tmp_" << std::endl;
        // std::cout << control_mean_trajectory_tmp_ << std::endl;

        // predict_state_sequence(state_tmp_, control_mean_trajectory_tmp_);

        // solve(state_tmp_);

        // std::cout << "Updated Control Sequence:" << std::endl;
        // std::cout << updated_control_sequence_ << std::endl;

        // std::cout << "Collision Rate:" << std::endl;
        // std::cout << collision_rate_ << std::endl;

        // ------------------------------------------------------------------------------------------------------------------------

        // function approximate_gradient_log_posterior_batch
        // State state_tmp_;
        // state_tmp_ << 0.0, 0.0, 0.0, 1.0, 0.1;
        // std::cout << "state_tmp_" << std::endl;
        // std::cout << state_tmp_ << std::endl;

        // auto a = approximate_gradient_log_posterior_batch(state_tmp_);

        // std::cout << a << std::endl;

        // ------------------------------------------------------------------------------------------------------------------------

        // test function random_sampling

        // ControlSequence control_mean_sequence_tmp_ = Eigen::MatrixXd::Zero(
        //     prediction_horizon_ - 1, CONTROL_SPACE::dim
        // );
        // control_mean_sequence_tmp_ << 0.1, 0.2, 0.3;
        // // std::cout << control_mean_sequence_tmp_ << std::endl;

        // ControlCovarianceSequence control_covariance_sequence_tmp_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        //     prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
        // );
        // control_covariance_sequence_tmp_[0] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.1;
        // control_covariance_sequence_tmp_[1] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.2;
        // control_covariance_sequence_tmp_[2] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.3;
        // // std::cout << control_covariance_sequence_tmp_[0] << std::endl;
        // // std::cout << control_covariance_sequence_tmp_[1] << std::endl;
        // // std::cout << control_covariance_sequence_tmp_[2] << std::endl;

        // // print member variable normal_distribution_pointer_
        // std::cout << "normal_distribution_pointer_\n";
        // for (size_t i = 0; i < normal_distribution_pointer_->size(); i++) {
        //     for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
        //         std::cout 
        //             << (*normal_distribution_pointer_)[i][j].mean()
        //             << ", "
        //             << (*normal_distribution_pointer_)[i][j].stddev()
        //             << std::endl;
        //     }
        // }

        // // print noise_sample_trajectory_batch_
        // std::cout << "noise_sequence_batch_:\n";
        // for (size_t i = 0; i < noise_sequence_batch_.size(); ++i) {
        //     std::cout << noise_sequence_batch_[i] << "\n";
        // }

        // // print noised_control_mean_sample_trajectory_batch_
        // std::cout << "noised_control_sequence_batch_:\n";
        // for (size_t i = 0; i < noised_control_sequence_batch_.size(); ++i) {
        //     std::cout << noised_control_sequence_batch_[i] << "\n";
        // }

        // random_sampling(control_mean_sequence_tmp_, control_covariance_sequence_tmp_);

        // // print member variable normal_distribution_pointer_
        // std::cout << "normal_distribution_pointer_\n";
        // for (size_t i = 0; i < normal_distribution_pointer_->size(); i++) {
        //     for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
        //         std::cout 
        //             << (*normal_distribution_pointer_)[i][j].mean()
        //             << ", "
        //             << (*normal_distribution_pointer_)[i][j].stddev()
        //             << std::endl;
        //     }
        // }

        // // print noise_sample_trajectory_batch_
        // std::cout << "noise_sequence_batch_:\n";
        // for (size_t i = 0; i < noise_sequence_batch_.size(); ++i) {
        //     std::cout << noise_sequence_batch_[i] << "\n";
        // }

        // // print noised_control_mean_sample_trajectory_batch_
        // std::cout << "noised_control_sequence_batch_:\n";
        // for (size_t i = 0; i < noised_control_sequence_batch_.size(); ++i) {
        //     std::cout << noised_control_sequence_batch_[i] << "\n";
        // }

        // ------------------------------------------------------------------------------------------------------------------------

        // approximate_gradient_log_likelihood

    }
};

} // namespace planning
} // namespace svg_mppi