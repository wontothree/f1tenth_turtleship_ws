#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {
namespace planning {

SVGMPPI::SVGMPPI() 
{
    const std::array<double, CONTROL_SPACE::dim> MIN_CONTROL = {MIN_STEERING};
    const std::array<double, CONTROL_SPACE::dim> MAX_CONTROL = {MAX_STEERING};

    // pointers
    guide_smapling_pointer_ = std::make_unique<Sampling>(
        GUIDE_SAMPLE_NUMBER,
        NON_BIASED_SAMPLING_RATE,
        PREDICTION_HORIZON,
        MIN_CONTROL,
        MAX_CONTROL,
        THREAD_NUMBER
    );
    prior_smapling_pointer_ = std::make_unique<Sampling>(
        SAMPLE_BATCH_NUMBER,
        NON_BIASED_SAMPLING_RATE,
        PREDICTION_HORIZON,
        MIN_CONTROL,
        MAX_CONTROL,
        THREAD_NUMBER
    );
    for (size_t i = 0; i < SAMPLE_BATCH_NUMBER; i++) {
        svg_sampling_pointer_.emplace_back(
            std::make_shared<Sampling>(
                SAMPLE_NUMBER_FOR_GRADIENT_ESTIMATION,
                NON_BIASED_SAMPLING_RATE,
                PREDICTION_HORIZON,
                MIN_CONTROL,
                MAX_CONTROL,
                THREAD_NUMBER,
                i
            )
        );
    }

    state_sequence_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        prior_smapling_pointer_->get_sample_number(), Eigen::MatrixXd::Zero(PREDICTION_HORIZON, STATE_SPACE::dim)
    );
    previous_control_sequence_ = Eigen::MatrixXd::Zero(
        PREDICTION_HORIZON - 1, CONTROL_SPACE::dim
    );
}

std::pair<ControlSequence, double> SVGMPPI::solve(
    const State& initial_state
)
{
    // Transport guide particles by sten variational gradient descent

    // cost batch
    std::vector<double> svg_cost_batch_history;
    std::vector<ControlSequence> svg_control_sequence_batch_history;

    for (size_t i = 0; i < SVGD_ITERATION_NUMBER; i++) {
        // Transport samples by stein variational gradient descent
        const ControlSequenceBatch gradient_log_posterior_batch = approximate_gradient_log_posterior_batch(
            *guide_smapling_pointer_,
            initial_state
        );

        #pragma omp parallel for num_threads(THREAD_NUMBER)
        for (size_t i = 0; i < guide_smapling_pointer_->get_sample_number(); i++) {
            // svgd update
            guide_smapling_pointer_->noised_control_sequence_batch_[i] += SVGD_STEP_SIZE * gradient_log_posterior_batch[i];
        }

        // store costs and samples for adaptive covariance calculation
        const std::vector<double> state_cost_batch = calculate_state_sequence_cost_batch(
            *guide_smapling_pointer_,
            initial_state,
            local_cost_map_
        ).first;
        svg_cost_batch_history.insert(
            svg_cost_batch_history.end(),
            state_cost_batch.begin(),
            state_cost_batch.end()
        );
        svg_control_sequence_batch_history.insert(
            svg_control_sequence_batch_history.end(),
            guide_smapling_pointer_->noised_control_sequence_batch_.begin(),
            guide_smapling_pointer_->noised_control_sequence_batch_.end()
        );
    }

    const auto guide_state_cost_batch = calculate_state_sequence_cost_batch(
        *guide_smapling_pointer_,
        initial_state,
        local_cost_map_
    ).first;
    const size_t min_guide_state_cost_index = std::distance(
        guide_state_cost_batch.begin(),
        std::min_element(
            guide_state_cost_batch.begin(),
            guide_state_cost_batch.end()
        )
    );
    const ControlSequence svg_control_sequence = guide_smapling_pointer_->noised_control_sequence_batch_[
        min_guide_state_cost_index
    ]; // Stein Variational Guided

    // ----------------------------------------------------------------------------------------------------

    // covariance matrix for random_sampling from prior distribution
    ControlCovarianceMatrixSequence adaptive_control_covariances_matrix_sequence = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        PREDICTION_HORIZON - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    for (auto& covariance : adaptive_control_covariances_matrix_sequence) {
        for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
            covariance(i, i) = STERRING_CONTROL_COVARIANCE[i];
        }
    }

    if (IS_COVARIANCE_ADAPTATION) {
        // calculate softmax costs
        const std::vector<double> softmax_costs = softmax(
            svg_cost_batch_history,
            GAUSSIAN_FITTING_LAMBDA,
            THREAD_NUMBER
        );

        // calculate covariance using gaussian fitting
        for (size_t i = 0; i < PREDICTION_HORIZON - 1; i++) {
            // x-coordinate vector for gaussian_fitting
            std::vector<double> steer_samples(svg_control_sequence_batch_history.size());
            // y-coordinate vector for gaussian_fitting
            std::vector<double> q_star(softmax_costs.size());
            
            for (size_t j = 0; j < steer_samples.size(); j++) {
                steer_samples[j] = svg_control_sequence_batch_history[j](i, 0);
                q_star[j] = softmax_costs[j];
            }

            // Gaussian fitting
            const double sigma = gaussian_fitting(
                steer_samples,
                q_star
            ).second;

            // clamping
            const double sigma_clamped = std::clamp(
                sigma,
                MIN_STEERING_COVARIANCE,
                MAX_STEERING_COVARIANCE
            );

            adaptive_control_covariances_matrix_sequence[i] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * sigma_clamped;
        }
    }

    // important from now

    // random sampling from prior distribution
    prior_smapling_pointer_->random_sampling(previous_control_sequence_, adaptive_control_covariances_matrix_sequence);

    // Rollout samples and calculate costs
    auto [state_sequence_cost_batch, collision_cost_batch] = calculate_state_sequence_cost_batch(
        *prior_smapling_pointer_,
        initial_state,
        local_cost_map_
    );

    // calculate weights
    if (IS_SVG) {
        // with nominal sequence by SVG
        nominal_control_sequence_ = svg_control_sequence;
    } else {
        // without nominal sequence
        nominal_control_sequence_ = Eigen::MatrixXd::Zero(
            PREDICTION_HORIZON - 1, CONTROL_SPACE::dim
        );
    }

    const std::vector<double> weight_batch = softmax(
        calculate_sample_cost_batch(
            *prior_smapling_pointer_,
            LAMBDA,
            ALPHA,
            state_sequence_cost_batch,
            nominal_control_sequence_
        ),
        LAMBDA,
        THREAD_NUMBER
    );
    weight_batch_ = weight_batch;

    // update next control sequence by expection (weighted sum)
    ControlSequence updated_control_sequence = Eigen::MatrixXd::Zero(PREDICTION_HORIZON - 1, CONTROL_SPACE::dim);
    for (size_t i = 0; i < prior_smapling_pointer_->get_sample_number(); i++) {
        updated_control_sequence += weight_batch[i] * prior_smapling_pointer_->noised_control_sequence_batch_[i];
    }

    // for collision rate
    const int collision_number = std::count_if(
        collision_cost_batch.begin(),
        collision_cost_batch.end(),
        [](const double& cost) { return cost > 0.0; }
    );
    const double collision_rate = static_cast<double>(collision_number) / static_cast<double>(prior_smapling_pointer_->get_sample_number());

    // for warm start (biased-sampling)
    previous_control_sequence_ = updated_control_sequence;

    return std::make_pair(updated_control_sequence, collision_rate);
}

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
    const Sampling& sampling_pointer,
    const State& initial_state,
    const grid_map::GridMap& local_cost_map
)
{
    // declare variables to return
    std::vector<double> state_sequence_cost_batch(sampling_pointer.get_sample_number());
    std::vector<double> collision_cost_batch(sampling_pointer.get_sample_number());

    #pragma omp parallel for num_threads(THREAD_NUMBER)
    for (size_t i = 0; i < sampling_pointer.get_sample_number(); i++) {
        // rollout state sequence for each control sequence
        state_sequence_batch_.at(i) = predict_state_sequence(
            initial_state,
            sampling_pointer.noised_control_sequence_batch_[i]
        );

        // calculate state sequence cost
        const auto [state_sequence_cost, collision_cost] = calculate_state_sequence_cost(
            state_sequence_batch_.at(i),
            local_cost_map
        );

        state_sequence_cost_batch.at(i) = state_sequence_cost;
        collision_cost_batch.at(i) = collision_cost;
    }

    return std::make_pair(state_sequence_cost_batch, collision_cost_batch);
}

std::vector<double> SVGMPPI::calculate_sample_cost_batch(
    const Sampling& sampling_pointer,
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
    for (size_t i = 0; i < sampling_pointer.get_sample_number(); i++) {
        for (size_t j = 0; j < PREDICTION_HORIZON - 1; j++) {
            const double control_cost = \
                lambda * (1 - alpha) \
                * (sampling_pointer.control_mean_sequence_.row(j) - nominal_control_sequence.row(j)) \
                * sampling_pointer.control_inverse_covariance_matrix_sequence_[j] \
                * sampling_pointer.noised_control_sequence_batch_[i].row(j).transpose();
            
            sample_cost_batch[i] += control_cost;
        }
    }

    return sample_cost_batch;
}

ControlSequenceBatch SVGMPPI::approximate_gradient_log_posterior_batch(
    const Sampling& sampling_pointer,
    const State& initial_state
)
{
    // declare and initialize gradient_log_posterior_batch
    ControlSequenceBatch gradient_log_posterior_batch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sampling_pointer.get_sample_number(), Eigen::MatrixXd::Zero(PREDICTION_HORIZON, STATE_SPACE::dim)
    );

    #pragma omp parallel for num_threads(THREAD_NUMBER)
    for (size_t i = 0; i < sampling_pointer.get_sample_number(); i++) {
        const ControlSequence gradient_log_likelihood = approximate_gradient_log_likelihood(
            svg_sampling_pointer_.at(i).get(),
            initial_state,
            sampling_pointer.noised_control_sequence_batch_[i],
            sampling_pointer.control_inverse_covariance_matrix_sequence_
        );

        gradient_log_posterior_batch[i] = gradient_log_likelihood;
    }

    return gradient_log_posterior_batch;
}

ControlSequence SVGMPPI::approximate_gradient_log_likelihood(
    Sampling* sampling_pointer,
    const State& initial_state,
    const ControlSequence& reference_control_sequence,
    const ControlCovarianceMatrixSequence& control_inverse_covariance_matrix_sequence
)
{
    // declate and initialize reference_control_covariance_matrix_sequence to be used random_sampling
    ControlCovarianceMatrixSequence reference_control_covariance_matrix_sequence = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        PREDICTION_HORIZON - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    for (auto& control_covariance : reference_control_covariance_matrix_sequence) {
        for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
            control_covariance(i, i) = steering_control_covariance_for_gradient_estimation_[i];
        }
    }

    // Generate control sequence samples and update member variable noised_control_sequence_batch_
    sampling_pointer->random_sampling(reference_control_sequence, reference_control_covariance_matrix_sequence);

    // calculate state cost for each state sequence
    auto state_cost_batch = calculate_state_sequence_cost_batch(
        *sampling_pointer,
        initial_state,
        local_cost_map_
    ).first;

    // declare variable for summing sample weight in denumerator
    std::vector<double> sample_weight_batch(sampling_pointer->get_sample_number());

    // declare and initialize variable for numerator term
    ControlSequence numerator = Eigen::MatrixXd::Zero(
        PREDICTION_HORIZON - 1, CONTROL_SPACE::dim
    );

    #pragma omp parallel for num_threads(THREAD_NUMBER)
    for (size_t i = 0; i < sampling_pointer->get_sample_number(); i++) {
        // declare variable for calculate sample cost (state cost + control cost) and initialize as state cost
        double sample_cost = state_cost_batch[i];

        // calculate sample cost
        for (size_t j = 0; j < PREDICTION_HORIZON - 1; j++) {
            const double control_cost = SVG_LAMBDA \
                * (previous_control_sequence_.row(j) - sampling_pointer->noised_control_sequence_batch_[i].row(j)) \
                * control_inverse_covariance_matrix_sequence[j] \
                * (previous_control_sequence_.row(j) - sampling_pointer->noised_control_sequence_batch_[i].row(j)).transpose();
            sample_cost += control_cost;
        }

        // calculate sample weight for each sample
        const double sample_weight = std::exp(-sample_cost / SVG_LAMBDA);

        // for denumerator
        sample_weight_batch[i] = sample_weight;
        
        // declare and initialize
        ControlSequence gradient_log_gaussian(reference_control_sequence.rows(), reference_control_sequence.cols());
        gradient_log_gaussian.setZero();
        
        for (size_t j = 0; j < PREDICTION_HORIZON - 1; j++) {
            gradient_log_gaussian.row(j) = sample_weight \
                * sampling_pointer->control_inverse_covariance_matrix_sequence_[j] \
                * (sampling_pointer->noised_control_sequence_batch_[i] - reference_control_sequence).row(j).transpose();
        }
        numerator += gradient_log_gaussian;
    }

    // denumberator term
    const double denumerator = std::accumulate(sample_weight_batch.begin(), sample_weight_batch.end(), 0.0);
    
    return numerator / (denumerator + 1e-10);
}

} // namespace planning
} // namespace svg_mppi
