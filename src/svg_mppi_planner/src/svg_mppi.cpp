#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {
namespace planning {

SVGMPPI::SVGMPPI() 
{
    control_mean_sequence_ = Eigen::MatrixXd::Zero(
        prediction_horizon_ - 1, CONTROL_SPACE::dim
    );
    control_covariance_sequence_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );

    noise_sequence_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim)
    );
    noised_control_sequence_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim)
    );

    state_sequence_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_horizon_, STATE_SPACE::dim)
    );
    control_inverse_covariance_sequence_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    previous_control_mean_sequence_ = Eigen::MatrixXd::Zero(
        prediction_horizon_, STATE_SPACE::dim
    );

    costs_ = std::vector<double>(sample_number_, 0.0);
    nominal_control_sequence_ = Eigen::MatrixXd::Zero(
        prediction_horizon_ - 1, CONTROL_SPACE::dim
    );

    test();
}

std::pair<ControlSequence, double> SVGMPPI::solve(
    const State& initial_state
)
{
    // Transport guide particles by sten variational gradient descent
    std::vector<double> costs_history_;
    std::vector<ControlSequence> control_sequence_history_;

    for (size_t i = 0; i < svgd_iteration_number_; i++) {
        // Transport samples by stein variational gradient descent
        const ControlSequenceBatch gradient_log_posterior_batch_ = approximate_gradient_log_posterior_batch(
            initial_state
        );

        // #pragma omp parallel for num_threads(thread_number_)
        // for (size_t i = 0; i < guide_sample_number_; i++) {
        //     // update
        //     noised_control_sequence_batch_[i] += svgd_step_size_ * gradient_log_posterior_batch_[i];
        // }

        // // store costs and samples for adaptive covariance calculation
        // const std::vector<double> state_cost_batch = calculate_state_cost_batch(
        //     initial_state,
        //     local_cost_map_,
        //     &state_sequence_batch_,
        //     noised_control_sequence_batch_
        // ).first;

        // costs_history_.insert(
        //     costs_history_.end(),
        //     state_cost_batch.begin(),
        //     state_cost_batch.end()
        // );

        // control_sequence_history_.insert(
        //     control_sequence_history_.end(),
        //     noised_control_sequence_batch_.begin(),
        //     noised_control_sequence_batch_.end()
        // );
    }

    // const auto guide_state_cost_batch_ = calculate_state_cost_batch(
    //     initial_state,
    //     local_cost_map_,
    //     &state_sequence_batch_,
    //     noised_control_sequence_batch_
    // ).first;

    // const size_t min_index_ = std::distance(
    //     guide_state_cost_batch_.begin(),
    //     std::min_element(
    //         guide_state_cost_batch_.begin(),
    //         guide_state_cost_batch_.end()
    //     )
    // );

    // const ControlSequence best_particle_ = noised_control_sequence_batch_[min_index_];

    // ControlCovarianceSequence covariances_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
    //     prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    // );

    // for (auto& covariance : covariances_) {
    //     for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
    //         covariance(i, i) = steering_control_covariance_for_gradient_estimation_[i];
    //     }
    // }

    // if (1) {
    //     // calculate softmax costs
    //     const std::vector<double> softmax_costs = softmax(
    //         costs_history_,
    //         gaussian_fitting_lambda_,
    //         thread_number_
    //     );

    //     // calculate covariance using gaussian fitting
    //     for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
    //         std::vector<double> steer_samples(control_sequence_history_.size());
    //         std::vector<double> q_star(softmax_costs.size());
    //         for (size_t j = 0; j < steer_samples.size(); j++) {
    //             steer_samples[j] = control_sequence_history_[j](i, 0);
    //             q_star[j] = control_sequence_history_[j](i, 0);
    //         }

    //         const double sigma = gaussian_fitting(
    //             steer_samples,
    //             q_star
    //         ).second;
    //         const double sigma_clamped = std::clamp(
    //             sigma,
    //             min_steering_covariance_,
    //             max_steering_covariance_
    //         );

    //         covariances_[i] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * sigma_clamped;
    //     }
    // }

    // // random sampling from prior distribution
    // random_sampling(previous_control_mean_sequence_, covariances_);

    // // Rollout samples and calculate costs
    // auto [_costs, _collision_costs] = calculate_state_cost_batch(
    //     initial_state,
    //     local_cost_map_,
    //     &state_sequence_batch_,
    //     noised_control_sequence_batch_
    // );
    // costs_ = std::forward<std::vector<double>>(_costs);

    // // calculate weights
    // if (1) {
    //     // with nominal sequence
    //     nominal_control_sequence_ = best_particle_;
    // } else {
    //     // without nominal sequence
    // }

    // const std::vector<double> weight_batch_ = softmax(
    //     calculate_sample_cost_batch(
    //         lambda_,
    //         alpha_,
    //         costs_,
    //         control_mean_sequence_,
    //         nominal_control_sequence_,
    //         noised_control_sequence_batch_,
    //         control_inverse_covariance_sequence_
    //     ),
    //     lambda_,
    //     thread_number_
    // );

    // ControlSequence updated_control_sequence_ = Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim);
    // for (size_t i = 0; i < sample_number_; i++) {
    //     updated_control_sequence_ += weight_batch_[i] * noised_control_sequence_batch_[i];
    // }

    // const int collision_number_ = std::count_if(
    //     _collision_costs.begin(),
    //     _collision_costs.end(),
    //     [](const double& cost) { return cost > 0.0; }
    // );
    // const double collision_rate_ = static_cast<double>(collision_number_) / static_cast<double>(sample_number_);

    // return std::make_pair(updated_control_sequence_, collision_rate_);
    return std::make_pair(control_mean_sequence_, 3.0);
}

ControlSequenceBatch SVGMPPI::approximate_gradient_log_posterior_batch(
    const State& initial_state
)
{
    ControlSequenceBatch gradient_log_posterior_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim)
    );

    const ControlSequence _control_mean_sequence_ = control_mean_sequence_;

    // for (size_t i = 0; i < sample_number_; i++) {
    //     const ControlSequence gradient_log_likelihood_ = approximate_gradient_log_likelihood(
    //         initial_state,
    //         _control_mean_sequence_,
    //         noised_control_sequence_batch_[i],
    //         control_inverse_covariance_sequence_
    //     );

    //     gradient_log_posterior_batch_[i] = gradient_log_likelihood_;
    // }

    return gradient_log_posterior_batch_;
}
ControlSequence SVGMPPI::approximate_gradient_log_likelihood(
    const State& initial_state,
    const ControlSequence& control_mean_sequence,
    const ControlSequence& noised_control_mean_sequence,
    const ControlCovarianceSequence& control_inverse_covariance_sequence
)
{
    ControlCovarianceSequence gradient_covariance_sequence_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    for (auto& gradient_covariance : gradient_covariance_sequence_) {
        for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
            gradient_covariance(i, i) = steering_control_covariance_for_gradient_estimation_[i];
        }
    }

    // Generate gaussian random samples, center of which is input_seq
    random_sampling(noised_control_mean_sequence, gradient_covariance_sequence_);

    // calculate forward simulation and costs
    auto cost_batch_ = calculate_state_cost_batch(
        initial_state,
        local_cost_map_,
        &state_sequence_batch_,
        noised_control_sequence_batch_
    ).first;

    //calculate cost with control term
    std::vector<double> exponential_cost_batch_(sample_number_);
    ControlSequence sum_of_grads = control_mean_sequence * 0.0;
    const ControlCovarianceSequence sampler_inverse_covariance = control_inverse_covariance_sequence_;
    #pragma omp parallel for num_threads(thread_number_)
    for (size_t i = 0; i < sample_number_; i++) {
        double cost_with_control_term = cost_batch_[i];
        for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
            const double diff_control_term = grad_lambda_ \
                * (previous_control_mean_sequence_.row(j) - noised_control_sequence_batch_[i].row(j)) \
                * control_inverse_covariance_sequence[j] \
                * (previous_control_mean_sequence_.row(j) - noised_control_sequence_batch_[i].row(j)).transpose();
            cost_with_control_term += diff_control_term;
        }
        const double exponential_cost_ = std::exp(-cost_with_control_term / grad_lambda_);
        exponential_cost_batch_[i] = exponential_cost_;
        
        ControlSequence gradient_log_gaussian_(control_mean_sequence.rows(), control_mean_sequence.cols());
        gradient_log_gaussian_.setZero();
        for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
            gradient_log_gaussian_.row(j) = exponential_cost_ \
                * sampler_inverse_covariance[j] \
                * (noised_control_sequence_batch_[i] - noised_control_mean_sequence).row(j).transpose();
        }
        sum_of_grads += gradient_log_gaussian_;
    }

    const double sum_of_costs = std::accumulate(exponential_cost_batch_.begin(), exponential_cost_batch_.end(), 0.0);

    return sum_of_grads / (sum_of_costs + 1e-10);
}

void SVGMPPI::random_sampling(
    const ControlSequence control_mean_sequence,
    const ControlCovarianceSequence control_covariance_sequence
)
{
    set_control_mean_sequence(control_mean_sequence);
    set_control_covariance_sequence(control_covariance_sequence);

    // Set normal distributions parameters
    for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            const double standard_deviation_ = std::sqrt(control_covariance_sequence_[i](j, j));
            std::normal_distribution<>::param_type param(0.0, standard_deviation_);
        }
    }

    #pragma omp parallel for num_threads(thread_number_)
    for (size_t i = 0; i < sample_number_; i++) {
        // generate noise sequence
        for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
            for (size_t k = 0; k < CONTROL_SPACE::dim; k++) {
                noise_sequence_batch_[i](j, k) = (*normal_distribution_pointer_)[j][k](random_number_generators_[omp_get_thread_num()]);
            }
        }

        // sampling control trajectory with non-biased (around zero) sampling rate
        if (i < static_cast<size_t>((1 - non_biased_sampling_rate_) * sample_number_)) {
            // biased sampling
            noised_control_sequence_batch_[i] = control_mean_sequence_ + noise_sequence_batch_[i];
        } else {
            // non-biased sampling (around zero)
            noised_control_sequence_batch_[i] = noise_sequence_batch_[i];
        }

        // clip input with control input constraints
        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
                noised_control_sequence_batch_[i](k, j) = std::clamp(
                    noised_control_sequence_batch_[i](k, j),
                    min_control_[j],
                    max_control_[j]
                );
            }
        }
    }
}

std::pair<std::vector<double>, std::vector<double>> SVGMPPI::calculate_state_cost_batch(
    const State& initial_state,
    const grid_map::GridMap& local_cost_map,
    StateSequenceBatch* state_sequence_batch,
    ControlSequenceBatch& control_sequence_batch
) const
{
    std::vector<double> total_cost_batch_(sample_number_);
    std::vector<double> collision_cost_batch_(sample_number_);

    #pragma omp parallel for num_threads(thread_number_)
    for (size_t i = 0; i < sample_number_; i++) {
        // predict state sequence
        state_sequence_batch->at(i) = predict_state_sequence(
            initial_state,
            control_sequence_batch[i]
        );

        // calculate state sequence cost
        const auto [total_cost_, collision_cost_] = calculate_state_sequence_cost(
            state_sequence_batch->at(i),
            local_cost_map
        );
        total_cost_batch_.at(i) = total_cost_;
        collision_cost_batch_.at(i) = collision_cost_;
    }

    return std::make_pair(total_cost_batch_, collision_cost_batch_);
}
StateSequence SVGMPPI::predict_state_sequence(
    const State& initial_state,
    const ControlSequence& control_sequence
) const
{
    // initialize state trajectory
    StateSequence predicted_state_sequence_ = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);

    // set current state to state trajectory as initial state
    predicted_state_sequence_.row(0) = initial_state;

    for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
        double steering_ = control_sequence(i, CONTROL_SPACE::steering);

        const double predicted_x_ = predicted_state_sequence_(i, STATE_SPACE::x);
        const double predicted_y_ = predicted_state_sequence_(i, STATE_SPACE::y);;
        const double predicted_yaw_ = predicted_state_sequence_(i, STATE_SPACE::yaw);
        const double predicted_velocity_ = predicted_state_sequence_(i, STATE_SPACE::velocity);
        const double predicted_steering_ = predicted_state_sequence_(i, STATE_SPACE::steering);

        // kinematic bicycle model
        const double sideslip_ = atan(lf_ / (lf_ + lr_) * tan(steering_));
        const double predicted_delta_x_ = predicted_velocity_ * cos(predicted_yaw_ + sideslip_) * prediction_interval_;
        const double predicted_delta_y_ = predicted_velocity_ * sin(predicted_yaw_ + sideslip_) * prediction_interval_;
        const double predicted_delta_yaw_ = predicted_velocity_ * sin(sideslip_) / lr_ * prediction_interval_;
        const double predicted_delta_steering_ = predicted_steering_;

        double next_velocity_ = 0.0;
        if (1) {
            next_velocity_ = predict_constant_speed(predicted_velocity_);
        }

        // next state
        predicted_state_sequence_(i + 1, STATE_SPACE::x) = predicted_x_ + predicted_delta_x_;
        predicted_state_sequence_(i + 1, STATE_SPACE::y) = predicted_y_ + predicted_delta_y_;
        predicted_state_sequence_(i + 1, STATE_SPACE::yaw) = std::atan2(sin(predicted_yaw_ + predicted_delta_yaw_), cos(predicted_yaw_ + predicted_delta_yaw_));
        predicted_state_sequence_(i + 1, STATE_SPACE::velocity) = next_velocity_;
        predicted_state_sequence_(i + 1, STATE_SPACE::steering) = predicted_steering_ + predicted_delta_steering_;
    }

    return predicted_state_sequence_;
}
std::pair<double, double> SVGMPPI::calculate_state_sequence_cost(
    const StateSequence state_sequence,
    const grid_map::GridMap& local_cost_map
) const
{
    // total cost of summing stage cost and terminal cost
    double state_squence_cost_sum_ = 0.0;

    // stage cost
    for (size_t i = 0; i < prediction_step_size_; i++) {
        double state_squence_stage_cost_sum_ = 10.0;

        State stage_state_ = state_sequence.row(i);
        if (local_cost_map.isInside(grid_map::Position(stage_state_(STATE_SPACE::x), stage_state_(STATE_SPACE::y)))) {
            state_squence_stage_cost_sum_ = local_cost_map.atPosition(
                "collision", grid_map::Position(stage_state_(STATE_SPACE::x), stage_state_(STATE_SPACE::y))
            );
        }

        state_squence_cost_sum_ += collision_weight_ * state_squence_stage_cost_sum_;
    }

    // terminal cost
    const State terminal_state_ = state_sequence.row(prediction_step_size_ - 1);
    double state_sequence_terminal_cost_sum_ = 10.0;
    if (local_cost_map.isInside(grid_map::Position(terminal_state_(STATE_SPACE::x), terminal_state_(STATE_SPACE::y)))) {
        state_sequence_terminal_cost_sum_ = local_cost_map.atPosition(
            "collision", grid_map::Position(terminal_state_(STATE_SPACE::x), terminal_state_(STATE_SPACE::y))
        );
    }

    state_squence_cost_sum_ += collision_weight_ * state_sequence_terminal_cost_sum_;

    return std::make_pair(state_squence_cost_sum_, state_squence_cost_sum_);
}

std::vector<double> SVGMPPI::calculate_sample_cost_batch(
    const double& lambda,
    const double& alpha,
    const std::vector<double> state_costs,
    const ControlSequence initial_consrol_sequence,
    const ControlSequence nominal_control_sequqnce,
    const ControlSequenceBatch control_sequence,
    const ControlCovarianceSequence control_inverse_covariance_sequence
) const
{
    // 모든 sample의 cost들
    std::vector<double> sample_costs = state_costs;

    #pragma omp parallel for num_threads(thread_number_)
    for (size_t i = 0; i < sample_number_; i++) {
        for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
            const double control_cost = \
                lambda * (1 - alpha) \
                * (initial_consrol_sequence.row(j) - nominal_control_sequqnce.row(j)) \
                * control_inverse_covariance_sequence[j] \
                * control_sequence[i].row(j).transpose();
            
            sample_costs[i] += control_cost;
        }
    }

    return sample_costs;
}

} // namespace planning
} // namespace svg_mppi
