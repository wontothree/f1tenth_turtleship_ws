// Anthony Garcia

#pragma once

#include <random>
#include <memory>

#include "svg_mppi_planner/common.hpp"

namespace svg_mppi {
namespace planning {

class Sampling {

private:
    // ----------------------------------------------------------------------------------------------------
    
    const size_t SAMPLE_NUMBER;
    const double NON_BIASED_SAMPLING_RATE;
    const size_t PREDICTION_HORIZON;
    const std::array<double, CONTROL_SPACE::dim> MAX_STEERING;
    const std::array<double, CONTROL_SPACE::dim> MIN_STEERING;
    const int THREAD_NUMBER;

    // ----------------------------------------------------------------------------------------------------

public:
    // for random_sampling
    std::vector<std::mt19937> random_number_generators_;
    std::unique_ptr<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>> normal_distribution_pointer_;
    const std::array<double, CONTROL_SPACE::dim> max_control_ = {MAX_STEERING};
    const std::array<double, CONTROL_SPACE::dim> min_control_ = {MIN_STEERING};
    ControlSequence control_mean_sequence_;
    ControlCovarianceMatrixSequence control_covariance_matrix_sequence_;
    ControlCovarianceMatrixSequence control_inverse_covariance_matrix_sequence_;
    ControlSequenceBatch noise_sequence_batch_;
    ControlSequenceBatch noised_control_sequence_batch_; // ***

public:
    Sampling(
        const size_t& sample_number,
        const double& non_biased_sampling_rate,
        const size_t& prediction_horizon,
        const std::array<double, CONTROL_SPACE::dim> min_control,
        const std::array<double, CONTROL_SPACE::dim> max_control,
        const int& thread_number,
        const int& seed = 79
    );
    ~Sampling() {};

    /**
     * @brief sampling several control sequences based on non-biased and biased sampling
     * @param reference_control_sequence to be used as mean for biased sampling
     * @param reference_control_covariance_matrix_sequence to be used covariance for biased sampling
     * 
     * update member variables
     * @note control_mean_sequence_ by setter function
     * @note control_covariance_matrix_sequence_ by setter function
     * @note normal_distribution_pointer_
     * @note noise_sequence_batch_
     * @note noised_control_sequence_batch_
     */
    void random_sampling(
        const ControlSequence reference_control_sequence,
        const ControlCovarianceMatrixSequence reference_control_covariance_matrix_sequence
    );
    
    void set_control_mean_sequence(
        const ControlSequence& control_mean_sequence
    )
    {
        control_mean_sequence_ = control_mean_sequence;
    }

    void set_control_covariance_matrix_sequence(
        const ControlCovarianceMatrixSequence& control_covariance_sequence
    )
    {
        control_covariance_matrix_sequence_ = control_covariance_sequence;

        // small value to add diagonal elements for preventing singular matrix
        const double epsilon_ = 1e-4;

        // calculate inverse of covariance matrices in advance to reduce computational cost
        for (size_t i = 0; i < PREDICTION_HORIZON - 1; i++) {
            control_inverse_covariance_matrix_sequence_[i] = control_covariance_matrix_sequence_[i].inverse() + epsilon_ * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
        }
    }

    size_t get_sample_number() const { return SAMPLE_NUMBER; }
};

} // snamespace planning
} // namespace svg_mppi