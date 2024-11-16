// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <random>
#include <omp.h>
#include <grid_map_core/GridMap.hpp>

#include "svg_mppi_planner/common.hpp"
#include "svg_mppi_planner/sampling.hpp"

namespace svg_mppi {
namespace planning {

class SVGMPPI {
// private:
public:
    // ----------------------------------------------------------------------------------------------------

    // Constant for Model Predictive Control
    const size_t PREDICTION_HORIZON = 15;
    const double PREDICTION_INTERVAL = 0.05; // s

    // Constant for MPPI
    const size_t SAMPLE_NUMBER = 10;
    const double NON_BIASED_SAMPLING_RATE = 0.1;
    const double COLLISION_WEIGHT = 1.0;

    // Constant for F1/10 vehicle
    const double L_F = 0.189;
    const double L_R = 0.135;
    const double MIN_STEERING = -0.45;
    const double MAX_STEERING = 0.45;
    const double MIN_STEERING_COVARIANCE = 0.001;
    const double MAX_STEERING_COVARIANCE = 0.1;

    // Constant for SVGD
    const size_t GUIDE_SAMPLE_NUMBER = 1;
    const size_t SVGD_ITERATION_NUMBER = 3;
    const double SVGD_STEP_SIZE = 0.005;
    const size_t SAMPLE_NUMBER_FOR_GRADIENT_ESTIMATION = 10; // shoud be = SAMPLE_NUMBER
    const double SVG_LAMBDA = 3.0;
    const size_t SAMPLE_BATCH_NUMBER = 100;
    const bool IS_COVARIANCE_ADAPTATION = true;
    const bool IS_SVG = true;
    const std::array<double, CONTROL_SPACE::dim> steering_control_covariance_for_gradient_estimation_ = {0.01};

    // etc
    const int THREAD_NUMBER = 4;
    const double gaussian_fitting_lambda_ = 0.1;
    const double GAUSSIAN_FITTING_LAMBDA = 0.1;
    const double LAMBDA = 3.0;
    const double ALPHA = 0.1;

    // ----------------------------------------------------------------------------------------------------

    // pointers
    std::unique_ptr<Sampling> guide_smapling_pointer_;
    std::unique_ptr<Sampling> prior_smapling_pointer_;
    std::vector<std::shared_ptr<Sampling>> svg_sampling_pointer_;

    // called by function calculate_state_sequence_cost_batch, and used for visualization
    StateSequenceBatch state_sequence_batch_; // ***

    // solve
    std::vector<double> costs_; // ?
    ControlSequence nominal_control_sequence_;

    // warm start
    ControlSequence previous_control_sequence_; // and for approximate_gradient_log_likelihood

    // local cost map
    grid_map::GridMap local_cost_map_;

public:
    SVGMPPI();
    ~SVGMPPI() {};

    /**
     * @brief stein variational guided mppi solver
     * @param initial_state
     * @return update_control_sequence
     * @return collision_rate
     */
    std::pair<ControlSequence, double> solve(
        const State& initial_state
    );

    /**
     * @brief set local cost map for planning
     * @param local_cost_map
     */
    void set_local_cost_map(
        grid_map::GridMap& local_cost_map
    )
    {
        local_cost_map_ = local_cost_map;
    }

private:
    /**
     * @brief predict state sequence using kinematic bicycle model
     * @param initial_state
     * @param control_sequence
     */
    StateSequence predict_state_sequence(
        const State& initial_state,
        const ControlSequence& control_sequence
    ) const;

    double predict_constant_speed(const double& current_speed) const { return current_speed; }

    /**
     * @brief calculate cost of given state sequence using only local_cost_map
     * @param state_sequence state sequence
     * @param local_cost_map local cost map
     */
    std::pair<double, double> calculate_state_sequence_cost(
        const StateSequence state_sequence,
        const grid_map::GridMap& local_cost_map
    ) const;

    /**
     * @brief calculate each state cost for all sample. always called after function random_sampling
     * @param sampling_pointer
     * @param initial_state
     * @param local_cost_map
     * 
     * update member variable
     * @note state_sequence_batch_ all sample of state sequence, can be used for visualization
     * 
     * used member variable
     * @note state_sequence_batch_
     * @note sampling_pointer.noised_control_sequence_batch_
     */
    std::pair<std::vector<double>, std::vector<double>> calculate_state_sequence_cost_batch(
        const Sampling& sampling_pointer,
        const State& initial_state,
        const grid_map::GridMap& local_cost_map
    );

    /**
     * @brief calculate final cost for all state sequence
     * @param lambda Regularization parameter
     * @param alpha Control input difference regularization
     * @param state_cost_batch state costs of all sample
     * @param nominal_control_sequence Nominal control sequence
     * @return A vector of weights for each trajectory
     * 
     * used member variable of class Sampling
     * @note sampling_pointer.control_mean_sequence_
     * @note sampling_pointer.control_inverse_covariance_matrix_sequence_
     * @note sampling_pointer.noised_control_sequence_batch_
     */
    std::vector<double> calculate_sample_cost_batch(
        const Sampling& sampling_pointer,
        const double& lambda,
        const double& alpha,
        const std::vector<double> state_cost_batch,
        const ControlSequence nominal_control_sequence
    ) const;





    /**
     * @brief calculate gradient batch
     * @param sampling_pointer
     * @param initial_state for function approximate_gradient_log_likelihood
     */
    ControlSequenceBatch approximate_gradient_log_posterior_batch(
        const Sampling& sampling_pointer,
        const State& initial_state
    );

    /**
     * @brief calculate one gradient
     * @param sampling_pointer pointer
     * @param initial_state for function calculate_state_sequence_cost_batch
     * @param reference_control_sequence for function random_sampling
     * @param control_inverse_covariance_sequence used in
     * 
     * update member variable
     * @note noise_sequence_batch_ in random_sampling
     * @note noised_control_sequence_batch_ in random_sampling
     * @note control_mean_sequence_ in random_sampling
     * @note control_covariance_matrix_sequence_ in random_sampling
     * @note control_inverse_covariance_matrix_sequence_ in random_sampling
     * 
     * used member variable
     * @note previous_control_sequence_
     * @note noised_control_sequence_batch_
     * @note control_inverse_covariance_matrix_sequence_
     */
    ControlSequence approximate_gradient_log_likelihood(
        Sampling* sampling_pointer,
        const State& initial_state,
        const ControlSequence& reference_control_sequence,
        const ControlCovarianceMatrixSequence& control_inverse_covariance_matrix_sequence
    );

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
     * @param x x-coordinate vector
     * @param y y-coordinate vector
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

    void test(
        const State& initial_state
    )
    {
        // ----------------------------------------------------------------------------------------------------

        // // test function random_sampling

        // control sequence
        ControlSequence reference_control_sequence_tmp = Eigen::MatrixXd::Zero(
            15 - 1, CONTROL_SPACE::dim
        );
        reference_control_sequence_tmp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // std::cout << "reference_control_sequence_tmp" << std::endl;
        // std::cout << reference_control_sequence_tmp << std::endl;

        // control covariance matrix 
        ControlCovarianceMatrixSequence reference_control_covariance_matrix_sequence_tmp = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            PREDICTION_HORIZON - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
        );
        reference_control_covariance_matrix_sequence_tmp[0] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[1] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[2] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[3] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[4] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[5] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[6] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[7] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[8] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[9] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[10] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[11] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[12] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        reference_control_covariance_matrix_sequence_tmp[13] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.0;
        // std::cout << "reference_control_covariance_matrix_sequence_tmp" << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[0] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[1] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[2] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[3] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[4] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[5] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[6] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[7] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[8] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[9] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[10] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[11] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[12] << std::endl;
        // std::cout << reference_control_covariance_matrix_sequence_tmp[13] << std::endl;

        // // before

        // // print member variable normal_distribution_pointer_
        // std::cout << "normal_distribution_pointer_\n";
        // for (size_t i = 0; i < prior_smapling_pointer_->normal_distribution_pointer_->size(); i++) {
        //     for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
        //         std::cout 
        //             << (*prior_smapling_pointer_->normal_distribution_pointer_)[i][j].mean()
        //             << ", "
        //             << (*prior_smapling_pointer_->normal_distribution_pointer_)[i][j].stddev()
        //             << std::endl;
        //     }
        // }

        // // print noise_sequence_batch_
        // std::cout << "noise_sequence_batch_:\n";
        // for (size_t i = 0; i < prior_smapling_pointer_->noise_sequence_batch_.size(); ++i) {
        //     std::cout << prior_smapling_pointer_->noise_sequence_batch_[i] << "\n";
        // }

        // // print noised_control_sequence_batch_
        // std::cout << "noised_control_sequence_batch_:\n";
        // for (size_t i = 0; i < prior_smapling_pointer_->noised_control_sequence_batch_.size(); ++i) {
        //     std::cout << prior_smapling_pointer_->noised_control_sequence_batch_[i] << "\n";
        // }

        // prior_smapling_pointer_->random_sampling(reference_control_sequence_tmp, reference_control_covariance_matrix_sequence_tmp);

        // // after

        // // print member variable normal_distribution_pointer_
        // std::cout << "normal_distribution_pointer_\n";
        // for (size_t i = 0; i < prior_smapling_pointer_->normal_distribution_pointer_->size(); i++) {
        //     for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
        //         std::cout 
        //             << (*prior_smapling_pointer_->normal_distribution_pointer_)[i][j].mean()
        //             << ", "
        //             << (*prior_smapling_pointer_->normal_distribution_pointer_)[i][j].stddev()
        //             << std::endl;
        //     }
        // }

        // // print noise_sequence_batch_
        // std::cout << "noise_sequence_batch_:\n";
        // for (size_t i = 0; i < prior_smapling_pointer_->noise_sequence_batch_.size(); ++i) {
        //     std::cout << prior_smapling_pointer_->noise_sequence_batch_[i] << "\n";
        // }

        // // print noised_control_sequence_batch_
        // std::cout << "noised_control_sequence_batch_:\n";
        // for (size_t i = 0; i < prior_smapling_pointer_->noised_control_sequence_batch_.size(); ++i) {
        //     std::cout << prior_smapling_pointer_->noised_control_sequence_batch_[i] << "\n";
        // }

        // ----------------------------------------------------------------------------------------------------

        // // test function predict_state_sequence & calculate_state_sequence_cost

        // State initial_state_tmp;
        // initial_state_tmp << 0.0, 0.0, 0.0, 1.0, 0.0;
        // std::cout << "initial_state_tmp" << std::endl;
        // std::cout << initial_state_tmp << std::endl;

        // ControlSequence control_sequence_tmp = Eigen::MatrixXd::Zero(
        //     PREDICTION_HORIZON - 1, CONTROL_SPACE::dim
        // );
        // control_sequence_tmp << 0.586023, -2.0311, 0.250938, 0.895655, -1.02602, 0.679724, 1.66303, 0.397099, 0.377119, -0.45, -0.45, 0.45, 0.45, -0.45;
        // std::cout << "control_sequence_tmp" << std::endl;
        // std::cout << control_sequence_tmp << std::endl;

        // StateSequence predicted_state_sequence = predict_state_sequence(
        //     initial_state_tmp, control_sequence_tmp
        // );

        // std::cout << "predicted_state_sequence" << std::endl;
        // std::cout << predicted_state_sequence << std::endl;

        // grid_map::GridMap* local_cost_map = &local_cost_map_;
        // Eigen::MatrixXf& costmapData = local_cost_map->get("collision_layer");
        // std::cout << "----------------------------------" << std::endl;
        // std::cout << costmapData << std::endl;
        // std::cout << "----------------------------------" << std::endl;

        // const auto [state_squence_cost_sum1, state_squence_cost_sum2] = calculate_state_sequence_cost(
        //     predicted_state_sequence,
        //     local_cost_map_
        // );

        // std::cout << state_squence_cost_sum1 << std::endl;

        // ----------------------------------------------------------------------------------------------------

        // // test calculate_state_sequence_cost_batch & calculate_sample_cost_batch

        // const auto [state_sequence_cost_batch, collision_cost_batch] = calculate_state_sequence_cost_batch(
        //     *prior_smapling_pointer_,
        //     initial_state,
        //     local_cost_map_
        // );

        // for (const auto& cost : state_sequence_cost_batch) {
        //     std::cout << cost << " ";
        // }
        // std::cout << std::endl;

        // const auto sample_cost_batch = calculate_sample_cost_batch(
        //     *prior_smapling_pointer_,
        //     LAMBDA,
        //     ALPHA,
        //     state_sequence_cost_batch,
        //     nominal_control_sequence_
        // );

        // for (const auto& cost : sample_cost_batch) {
        //     std::cout << cost << " ";
        // }
        // std::cout << std::endl;

        // ----------------------------------------------------------------------------------------------------

        // function approximate_gradient_log_likelihood

        // if (!svg_sampling_pointer_.at(0)) {
        //     throw std::runtime_error("svg_sampling_pointer_.at(0) is null!");
        // }

        // if (!prior_smapling_pointer_) {
        //     throw std::runtime_error("prior_smapling_pointer_ is null!");
        // }

        // approximate_gradient_log_likelihood(
        //     svg_sampling_pointer_.at(0).get(),
        //     initial_state,
        //     prior_smapling_pointer_->noised_control_sequence_batch_[0],
        //     reference_control_covariance_matrix_sequence_tmp
        // );

        // ----------------------------------------------------------------------------------------------------

        // function approximate_gradient_log_posterior_batch

        // approximate_gradient_log_posterior_batch(
        //     *guide_smapling_pointer_,
        //     initial_state
        // );

        // ----------------------------------------------------------------------------------------------------
    }

};

} // namespace planning
} // namespace svg_mppi