#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {
namespace planning {

SVGMPPI::SVGMPPI() 
{
    const std::array<double, CONTROL_SPACE::dim> MIN_CONTROL = {MIN_STEERING};
    const std::array<double, CONTROL_SPACE::dim> MAX_CONTROL = {MAX_STEERING};

    // pointers
    guide_smaple_pointer_ = std::make_unique<Sampling>(
        GUIDE_SAMPLE_NUMBER,
        NON_BIASED_SAMPLING_RATE,
        PREDICTION_HORIZON,
        MIN_CONTROL,
        MAX_CONTROL,
        THREAD_NUMBER
    );
    prior_smaple_pointer_ = std::make_unique<Sampling>(
        SAMPLE_NUMBER,
        NON_BIASED_SAMPLING_RATE,
        PREDICTION_HORIZON,
        MIN_CONTROL,
        MAX_CONTROL,
        THREAD_NUMBER
    );

    state_sequence_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        SAMPLE_NUMBER, Eigen::MatrixXd::Zero(PREDICTION_HORIZON, STATE_SPACE::dim)
    );
    previous_control_mean_sequence_ = Eigen::MatrixXd::Zero(
        PREDICTION_HORIZON, STATE_SPACE::dim
    );


    costs_ = std::vector<double>(SAMPLE_NUMBER, 0.0);
    nominal_control_sequence_ = Eigen::MatrixXd::Zero(
        PREDICTION_HORIZON - 1, CONTROL_SPACE::dim
    );
}

// std::pair<ControlSequence, double> SVGMPPI::solve(
//     const State& initial_state
// )
// {
//     // Transport guide particles by sten variational gradient descent
//     std::vector<double> costs_history_;
//     std::vector<ControlSequence> control_sequence_history_;

//     for (size_t i = 0; i < SVGD_ITERATION_NUMBER; i++) {
//         // Transport samples by stein variational gradient descent
//         const ControlSequenceBatch gradient_log_posterior_batch_ = approximate_gradient_log_posterior_batch(
//             initial_state
//         );

//         #pragma omp parallel for num_threads(THREAD_NUMBER)
//         for (size_t i = 0; i < GUIDE_SAMPLE_NUMBER; i++) {
//             // svgd update
//             guide_noised_control_sequence_batch_[i] += SVGD_STEP_SIZE * gradient_log_posterior_batch_[i];
//         }

//         // store costs and samples for adaptive covariance calculation
//         const std::vector<double> state_cost_batch = calculate_state_sequence_cost_batch(
//             initial_state,
//             local_cost_map_
//         ).first;

//         costs_history_.insert(
//             costs_history_.end(),
//             state_cost_batch.begin(),
//             state_cost_batch.end()
//         );

//         control_sequence_history_.insert(
//             control_sequence_history_.end(),
//             noised_control_sequence_batch_.begin(),
//             noised_control_sequence_batch_.end()
//         );
//     }

//     const auto guide_state_cost_batch_ = calculate_state_sequence_cost_batch(
//         initial_state,
//         local_cost_map_
//     ).first;

//     const size_t min_index_ = std::distance(
//         guide_state_cost_batch_.begin(),
//         std::min_element(
//             guide_state_cost_batch_.begin(),
//             guide_state_cost_batch_.end()
//         )
//     );

//     const ControlSequence best_particle_ = noised_control_sequence_batch_[min_index_];

//     ControlCovarianceMatrixSequence covariances_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
//         PREDICTION_HORIZON - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
//     );

//     for (auto& covariance : covariances_) {
//         for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
//             covariance(i, i) = steering_control_covariance_for_gradient_estimation_[i];
//         }
//     }

//     if (1) {
//         // calculate softmax costs
//         const std::vector<double> softmax_costs = softmax(
//             costs_history_,
//             gaussian_fitting_lambda_,
//             THREAD_NUMBER
//         );

//         // calculate covariance using gaussian fitting
//         for (size_t i = 0; i < PREDICTION_HORIZON - 1; i++) {
//             std::vector<double> steer_samples(control_sequence_history_.size());
//             std::vector<double> q_star(softmax_costs.size());
//             for (size_t j = 0; j < steer_samples.size(); j++) {
//                 steer_samples[j] = control_sequence_history_[j](i, 0);
//                 q_star[j] = control_sequence_history_[j](i, 0);
//             }

//             const double sigma = gaussian_fitting(
//                 steer_samples,
//                 q_star
//             ).second;
//             const double sigma_clamped = std::clamp(
//                 sigma,
//                 MIN_STEERING_COVARIANCE,
//                 MAX_STEERING_COVARIANCE
//             );

//             covariances_[i] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * sigma_clamped;
//         }
//     }

//     // random sampling from prior distribution
//     random_sampling(previous_control_mean_sequence_, covariances_);

//     // Rollout samples and calculate costs
//     auto [_costs, _collision_costs] = calculate_state_sequence_cost_batch(
//         initial_state,
//         local_cost_map_
//     );
//     costs_ = std::forward<std::vector<double>>(_costs);

//     // calculate weights
//     if (1) {
//         // with nominal sequence
//         nominal_control_sequence_ = best_particle_;
//     } else {
//         // without nominal sequence
//     }

//     const std::vector<double> weight_batch_ = softmax(
//         calculate_sample_cost_batch(
//             LAMBDA,
//             ALPHA,
//             costs_,
//             nominal_control_sequence_
//         ),
//         LAMBDA,
//         THREAD_NUMBER
//     );

//     ControlSequence updated_control_sequence_ = Eigen::MatrixXd::Zero(PREDICTION_HORIZON - 1, CONTROL_SPACE::dim);
//     for (size_t i = 0; i < SAMPLE_NUMBER; i++) {
//         updated_control_sequence_ += weight_batch_[i] * noised_control_sequence_batch_[i];
//     }

//     const int collision_number_ = std::count_if(
//         _collision_costs.begin(),
//         _collision_costs.end(),
//         [](const double& cost) { return cost > 0.0; }
//     );
//     const double collision_rate_ = static_cast<double>(collision_number_) / static_cast<double>(SAMPLE_NUMBER);

//     // return std::make_pair(updated_control_sequence_, collision_rate_);

//     test(initial_state);

//     // temp
//     return std::make_pair(control_mean_sequence_, 3.0);
// }


StateSequence SVGMPPI::predict_state_sequence(
    const State& initial_state,
    const ControlSequence& control_sequence
) const
{
    // initialize state trajectory
    StateSequence predicted_state_sequence = Eigen::MatrixXd::Zero(PREDICTION_HORIZON, STATE_SPACE::dim);

    // set current state to state trajectory as initial state
    predicted_state_sequence.row(0) = initial_state;

    for (size_t i = 0; i < PREDICTION_HORIZON - 1; i++) {
        double steering = control_sequence(i, CONTROL_SPACE::steering);

        const double predicted_x = predicted_state_sequence(i, STATE_SPACE::x);
        const double predicted_y = predicted_state_sequence(i, STATE_SPACE::y);;
        const double predicted_yaw = predicted_state_sequence(i, STATE_SPACE::yaw);
        const double predicted_velocity = predicted_state_sequence(i, STATE_SPACE::velocity);
        const double predicted_steering = predicted_state_sequence(i, STATE_SPACE::steering);

        // predict delta state using kinematic bicycle model
        const double sideslip = atan(L_R / (L_F + L_R) * tan(steering));
        const double predicted_delta_x = predicted_velocity * cos(predicted_yaw + sideslip) * PREDICTION_INTERVAL;
        const double predicted_delta_y = predicted_velocity * sin(predicted_yaw + sideslip) * PREDICTION_INTERVAL;
        // const double predicted_delta_yaw = predicted_velocity * sin(sideslip) / L_R * PREDICTION_INTERVAL;
        const double predicted_delta_yaw = predicted_velocity / (L_F + L_R) * cos(sideslip) * tan(predicted_yaw) * PREDICTION_INTERVAL; // check
        const double predicted_delta_steering = (steering - predicted_steering) * PREDICTION_INTERVAL; // steer 1st order delay // check

        double next_velocity = 0.0;
        if (1) {
            next_velocity = predict_constant_speed(predicted_velocity);
        }

        // update next state
        predicted_state_sequence(i + 1, STATE_SPACE::x) = predicted_x + predicted_delta_x;
        predicted_state_sequence(i + 1, STATE_SPACE::y) = predicted_y + predicted_delta_y;
        predicted_state_sequence(i + 1, STATE_SPACE::yaw) = std::atan2(sin(predicted_yaw + predicted_delta_yaw), cos(predicted_yaw + predicted_delta_yaw));
        predicted_state_sequence(i + 1, STATE_SPACE::velocity) = next_velocity;
        predicted_state_sequence(i + 1, STATE_SPACE::steering) = predicted_steering + predicted_delta_steering;
    }

    return predicted_state_sequence;
}

std::pair<double, double> SVGMPPI::calculate_state_sequence_cost(
    const StateSequence state_sequence,
    const grid_map::GridMap& local_cost_map
) const
{
    // initialize total cost of summing stage cost and terminal cost
    double state_squence_cost_sum = 0.0;

    // stage cost
    for (size_t i = 0; i < PREDICTION_HORIZON - 1; i++) {
        double state_squence_stage_cost = 10.0;

        State stage_state = state_sequence.row(i);
        if (local_cost_map.isInside(grid_map::Position(stage_state(STATE_SPACE::x), stage_state(STATE_SPACE::y)))) {
            state_squence_stage_cost = local_cost_map.atPosition(
                "collision_layer", grid_map::Position(stage_state(STATE_SPACE::x), stage_state(STATE_SPACE::y))
            );
        }

        state_squence_cost_sum += COLLISION_WEIGHT * state_squence_stage_cost;
    }

    // terminal cost
    const State terminal_state = state_sequence.row(PREDICTION_HORIZON - 1);
    double state_sequence_terminal_cost = 10.0;
    if (local_cost_map.isInside(grid_map::Position(terminal_state(STATE_SPACE::x), terminal_state(STATE_SPACE::y)))) {
        state_sequence_terminal_cost = local_cost_map.atPosition(
            "collision_layer", grid_map::Position(terminal_state(STATE_SPACE::x), terminal_state(STATE_SPACE::y))
        );
    }

    state_squence_cost_sum += COLLISION_WEIGHT * state_sequence_terminal_cost;

    return std::make_pair(state_squence_cost_sum, state_squence_cost_sum);
}

std::pair<std::vector<double>, std::vector<double>> SVGMPPI::calculate_state_sequence_cost_batch(
    const Sampling& sample_pointer,
    const State& initial_state,
    const grid_map::GridMap& local_cost_map
)
{
    // declare variables to return
    std::vector<double> total_cost_batch(sample_pointer.get_sample_number());
    std::vector<double> collision_cost_batch(sample_pointer.get_sample_number());

    #pragma omp parallel for num_threads(THREAD_NUMBER)
    for (size_t i = 0; i < sample_pointer.get_sample_number(); i++) {
        // rollout state sequence for each control sequence
        state_sequence_batch_.at(i) = predict_state_sequence(
            initial_state,
            sample_pointer.noised_control_sequence_batch_[i]
        );

        // calculate state sequence cost
        const auto [total_cost, collision_cost] = calculate_state_sequence_cost(
            state_sequence_batch_.at(i),
            local_cost_map
        );

        total_cost_batch.at(i) = total_cost;
        collision_cost_batch.at(i) = collision_cost;
    }

    return std::make_pair(total_cost_batch, collision_cost_batch);
}

std::vector<double> SVGMPPI::calculate_sample_cost_batch(
    const Sampling& sample_pointer,
    const double& lambda,
    const double& alpha,
    const std::vector<double> state_cost_batch,
    const ControlSequence nominal_control_sequence
) const
{
    // declare and initialize sample cost batch as state cost batch
    std::vector<double> sample_cost_batch = state_cost_batch;

    // add control cost batch to sample cost batch
    #pragma omp parallel for num_threads(THREAD_NUMBER)
    for (size_t i = 0; i < SAMPLE_NUMBER; i++) {
        for (size_t j = 0; j < PREDICTION_HORIZON - 1; j++) {
            const double control_cost = \
                lambda * (1 - alpha) \
                * (sample_pointer.control_mean_sequence_.row(j) - nominal_control_sequence.row(j)) \
                * sample_pointer.control_inverse_covariance_matrix_sequence_[j] \
                * sample_pointer.noised_control_sequence_batch_[i].row(j).transpose();
            
            sample_cost_batch[i] += control_cost;
        }
    }

    return sample_cost_batch;
}





// ControlSequenceBatch SVGMPPI::approximate_gradient_log_posterior_batch(
//     const State& initial_state
// )
// {
//     // declare and initialize gradient_log_posterior_batch
//     ControlSequenceBatch gradient_log_posterior_batch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
//         SAMPLE_NUMBER, Eigen::MatrixXd::Zero(PREDICTION_HORIZON, STATE_SPACE::dim)
//     );
//     for (size_t i = 0; i < GUIDE_SAMPLE_NUMBER; i++) {
//         const ControlSequence gradient_log_likelihood = approximate_gradient_log_likelihood(
//             initial_state,
//             guide_noised_control_sequence_batch_[i],
//             control_inverse_covariance_matrix_sequence_
//         );

//         gradient_log_posterior_batch[i] = gradient_log_likelihood;
//     }

//     return gradient_log_posterior_batch;
// }

// ControlSequence SVGMPPI::approximate_gradient_log_likelihood(
//     const State& initial_state,
//     const ControlSequence& reference_control_sequence,
//     const ControlCovarianceMatrixSequence& control_inverse_covariance_matrix_sequence
// )
// {
//     // declate and initialize reference_control_covariance_matrix_sequence to be used random_sampling
//     ControlCovarianceMatrixSequence reference_control_covariance_matrix_sequence = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
//         PREDICTION_HORIZON - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
//     );
//     for (auto& control_covariance : reference_control_covariance_matrix_sequence) {
//         for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
//             control_covariance(i, i) = steering_control_covariance_for_gradient_estimation_[i];
//         }
//     }

//     // Generate control sequence samples and update member variable noised_control_sequence_batch_
//     random_sampling(reference_control_sequence, reference_control_covariance_matrix_sequence);

//     // calculate state cost for each state sequence
//     auto state_cost_batch = calculate_state_sequence_cost_batch(
//         initial_state,
//         local_cost_map_
//     ).first;

//     // sample cost (state cost + control cost)
//     std::vector<double> sample_weight_batch(SAMPLE_NUMBER_FOR_GRADIENT_ESTIMATION);

//     // declare and initialize variable for numerator term
//     ControlSequence weighted_centered_difference_sum = Eigen::MatrixXd::Zero(
//         PREDICTION_HORIZON - 1, CONTROL_SPACE::dim
//     );
//     #pragma omp parallel for num_threads(THREAD_NUMBER)
//     for (size_t i = 0; i < SAMPLE_NUMBER_FOR_GRADIENT_ESTIMATION; i++) {
//         // calculate sample cost (state cost + control cost)
//         double sample_cost = state_cost_batch[i];
//         for (size_t j = 0; j < PREDICTION_HORIZON - 1; j++) {
//             const double control_cost = grad_lambda_ \
//                 * (previous_control_mean_sequence_.row(j) - noised_control_sequence_batch_[i].row(j)) \
//                 * control_inverse_covariance_matrix_sequence[j] \
//                 * (previous_control_mean_sequence_.row(j) - noised_control_sequence_batch_[i].row(j)).transpose();
//             sample_cost += control_cost;
//         }

//         const double sample_weight = std::exp(-sample_cost / grad_lambda_);
//         sample_weight_batch[i] = sample_weight;
        
//         ControlSequence gradient_log_gaussian(reference_control_sequence.rows(), reference_control_sequence.cols());
//         gradient_log_gaussian.setZero();
//         for (size_t j = 0; j < PREDICTION_HORIZON - 1; j++) {
//             gradient_log_gaussian.row(j) = sample_weight \
//                 * control_inverse_covariance_matrix_sequence_[j] \
//                 * (noised_control_sequence_batch_[i] - control_mean_sequence_).row(j).transpose();
//         }
//         weighted_centered_difference_sum += gradient_log_gaussian;
//     }

//     // denumberator term
//     const double weight_sum = std::accumulate(sample_weight_batch.begin(), sample_weight_batch.end(), 0.0);

//     return weighted_centered_difference_sum / (weight_sum + 1e-10);
// }

} // namespace planning
} // namespace svg_mppi
