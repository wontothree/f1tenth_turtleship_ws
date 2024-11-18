#include "svg_mppi_planner/sampling.hpp"

namespace svg_mppi {
namespace planning {

Sampling::Sampling(
    const size_t& sample_number,
    const double& non_biased_sampling_rate,
    const size_t& prediction_horizon,
    const std::array<double, CONTROL_SPACE::dim> min_control,
    const std::array<double, CONTROL_SPACE::dim> max_control,
    const int& thread_number,
    const int& seed
): SAMPLE_NUMBER(sample_number),
   NON_BIASED_SAMPLING_RATE(non_biased_sampling_rate),
   PREDICTION_HORIZON(prediction_horizon),
   MIN_STEERING(min_control),
   MAX_STEERING(max_control),
   THREAD_NUMBER(thread_number)
{
    noise_sequence_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        SAMPLE_NUMBER, Eigen::MatrixXd::Zero(PREDICTION_HORIZON - 1, CONTROL_SPACE::dim)
    );
    noised_control_sequence_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        SAMPLE_NUMBER, Eigen::MatrixXd::Zero(PREDICTION_HORIZON - 1, CONTROL_SPACE::dim)
    );
    control_mean_sequence_ = Eigen::MatrixXd::Zero(
        PREDICTION_HORIZON - 1, CONTROL_SPACE::dim
    );
    control_covariance_matrix_sequence_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        PREDICTION_HORIZON - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    control_inverse_covariance_matrix_sequence_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        PREDICTION_HORIZON - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );

    // initialize random number generator
    for (int i = 0; i < THREAD_NUMBER; i++) {
        random_number_generators_.push_back(std::mt19937(std::random_device{}()));
    }

    // initialzie normal distributions as standard normal distribution
    normal_distribution_pointer_ = std::make_unique<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>>();
    for (size_t i = 0; i < PREDICTION_HORIZON - 1; i++) {
        std::array<std::normal_distribution<>, CONTROL_SPACE::dim> normal_distributions_ = {};

        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            std::normal_distribution<> standard_normal_distribution_(0.0, 1.0);
            normal_distributions_[j] = standard_normal_distribution_;
        }
        (*normal_distribution_pointer_).push_back(normal_distributions_);
    }
}

void Sampling::random_sampling(
    const ControlSequence reference_control_sequence,
    const ControlCovarianceMatrixSequence reference_control_covariance_matrix_sequence
)
{
    set_control_mean_sequence(reference_control_sequence);
    set_control_covariance_matrix_sequence(reference_control_covariance_matrix_sequence);

    // Set normal distributions parameters and pointer
    for (size_t i = 0; i < PREDICTION_HORIZON - 1; i++) {
        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            // standard deviation of control covariance sequence
            const double standard_deviation = std::sqrt(control_covariance_matrix_sequence_[i](j, j));

            // normal distribution parameter in which expectation is 0 and standard deviation is from control covariance sequence
            std::normal_distribution<>::param_type normal_distribution_parameter(0.0, standard_deviation);

            // set noraml distribution pointer
            (*normal_distribution_pointer_)[i][j].param(normal_distribution_parameter);
        }
    }

    #pragma omp parallel for num_threads(THREAD_NUMBER)
    for (size_t i = 0; i < SAMPLE_NUMBER; i++) {
        // generate noise sequence using upper normal distribution
        for (size_t j = 0; j < PREDICTION_HORIZON - 1; j++) {
            for (size_t k = 0; k < CONTROL_SPACE::dim; k++) {
                noise_sequence_batch_[i](j, k) = (*normal_distribution_pointer_)[j][k](random_number_generators_[omp_get_thread_num()]);
            }
        }

        // sampling control sequence with non-biased (around zero) sampling rate
        if (i < static_cast<size_t>((1 - NON_BIASED_SAMPLING_RATE) * SAMPLE_NUMBER)) {
            // biased sampling (around control_mean_sequence)
            noised_control_sequence_batch_[i] = control_mean_sequence_ + noise_sequence_batch_[i];
        } else {
            // non-biased sampling (around zero)
            noised_control_sequence_batch_[i] = noise_sequence_batch_[i];
        }

        // clip input with control input constraints
        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            for (size_t k = 0; k < PREDICTION_HORIZON - 1; k++) {
                noised_control_sequence_batch_[i](k, j) = std::clamp(
                    noised_control_sequence_batch_[i](k, j),
                    min_control_[j],
                    max_control_[j]
                );
            }
        }
    }
}

} // snamespace planning
} // namespace svg_mppi
