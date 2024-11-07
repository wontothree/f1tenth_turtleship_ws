#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {
namespace planning {

SVGMPPI::SVGMPPI() 
{
    control_mean_trajectory_ = Eigen::MatrixXd::Zero(
        prediction_horizon_ - 1, CONTROL_SPACE::dim
    );
    control_covariance_trajectory_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    control_inverse_covariance_trajectory_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    noise_sample_trajectory_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim)
    );
    noised_control_mean_sample_trajectory_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim)
    );
    state_trajectory_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_horizon_, STATE_SPACE::dim)
    );
    previous_control_mean_trajectory_ = Eigen::MatrixXd::Zero(
        prediction_horizon_, STATE_SPACE::dim
    );


    // initialize random number generator
    for (int i = 0; i < thread_number_; i++) {
        random_number_generators_.push_back(std::mt19937(std::random_device{}()));
    }

    // initialzie normal distributions as standard normal distribution
    normal_distribution_pointer_ = std::make_unique<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>>();
    for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
        std::array<std::normal_distribution<>, CONTROL_SPACE::dim> normal_distributions_ = {};

        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            std::normal_distribution<> standard_normal_distribution_(0.0, 1.0);
            normal_distributions_[j] = standard_normal_distribution_;
        }
        (*normal_distribution_pointer_).push_back(normal_distributions_);
    }

    test();
}

std::pair<ControlMeanTrajectory, double> SVGMPPI::solve(
    const State& current_state
)
{
    // transport guide particles by stein variational gradient descent
    std::vector<double> cost_history;
    std::vector<ControlMeanTrajectory> control_trajectory_history;
    auto function_calculate_sample_cost_batch = [&]() {
        return calculate_sample_cost_batch(current_state, local_cost_map_, &state_trajectory_batch_).first;
    };

    for (int i = 0; i < num_svgd_iteration_; i++) {
        // transport samples by stein variational gradient descent
        const ControlMeanTrajectoryBatch gradient_log_posterior_ = approximate_gradient_log_posterior_batch(current_state);

        #pragma omp parallel for num_threads(thread_number_)
        for (size_t i = 0; i < sample_number_; i++) {
            // stein variational gradient descent
            noised_control_mean_sample_trajectory_batch_[i] += svgd_step_size_ * gradient_log_posterior_[i];
        }

        // store costs and samples for adaptive covariance calculation
        const std::vector<double> costs = calculate_sample_cost_batch(
            current_state, 
            local_cost_map_, 
            &state_trajectory_batch_
        ).first;

        cost_history.insert(
            cost_history.end(), 
            costs.begin(), 
            costs.end()
        );
        control_trajectory_history.insert(
            control_trajectory_history.end(),
            noised_control_mean_sample_trajectory_batch_.begin(),
            noised_control_mean_sample_trajectory_batch_.end()
        );
    }

    const auto guide_costs = calculate_sample_cost_batch(
        current_state,
        local_cost_map_, 
        &state_trajectory_batch_
    ).first;

    const size_t min_index_ = std::distance(
        guide_costs.begin(),
        std::min_element(guide_costs.begin(), guide_costs.end())
    );

    const ControlMeanTrajectory best_particle_ = noised_control_mean_sample_trajectory_batch_[min_index_];

    // calculate adaptive covariance matrices for prior distribution
    ControlCovarianceTrajectory covariances_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    for (auto& covariance : covariances_) {
        for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
            covariance(i, i) = steering_control_covariance_for_gradient_estimation_[i];
        }
    }

    if (1) {
        // calculate softmax costs
        const std::vector<double> softmax_costs = softmax(
            cost_history,
            gaussian_fitting_lambda_,
            thread_number_
        );

        // calculate covariance using gaussian fitting
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            std::vector<double> steer_samples(control_trajectory_history.size());
            std::vector<double> q_star(softmax_costs.size());
            for (size_t j = 0; j < steer_samples.size(); j++) {
                steer_samples[j] = control_trajectory_history[j](i, 0);
                q_star[j] = control_trajectory_history[j](i, 0);
            }

            const double sigma = gaussian_fitting(steer_samples, q_star).second;
            const double sigma_clamped = std::clamp(
                sigma, 
                min_steering_covariance_, 
                max_steering_covariance_
            );

            covariances_[i] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * sigma_clamped;
        }
    }

    // random sampling from prior distribution
    random_sampling(previous_control_mean_trajectory_, covariances_);

    // Rollout samples and calculate costs
    auto [_costs, _collision_costs] = calculate_sample_cost_batch(
        current_state,
        local_cost_map_,
        &state_trajectory_batch_
    );
}


std::pair<std::vector<double>, std::vector<double>> SVGMPPI::calculate_sample_cost_batch(
    const State& current_state,
    const grid_map::GridMap& local_cost_map,
    StateMeanTrajectoryBatch* state_trajectory_candidates
) const
{
    std::vector<double> cost_batch_(sample_number_);
    std::vector<double> collision_cost_batch_(sample_number_);

    #pragma omp parallel for num_threads(thread_number_)
    for (size_t i = 0; i < sample_number_; i++) {
        // predict state trajectory
        state_trajectory_candidates->at(i) = predict_state_trajectory(
            current_state,
            noised_control_mean_sample_trajectory_batch_[i]
        );

        // calculate cost
        const auto [cost_, collision_cost_] = calculate_state_trajectory_cost(
            state_trajectory_candidates->at(i),
            local_cost_map
        );

        cost_batch_.at(i) = cost_;
        collision_cost_batch_.at(i) = collision_cost_;
    }

    return std::make_pair(cost_batch_, collision_cost_batch_);
}

StateMeanTrajectory SVGMPPI::predict_state_trajectory(
    const State& current_state,
    const ControlMeanTrajectory& control_trajectory
) const
{
    // initializer state trajectory as 0
    StateMeanTrajectory state_trajectory_ = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);

    // set current state to state trajectory as initial state
    state_trajectory_.row(0) = current_state;

    // 모든 horizon step을 순회한다.
    for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
        // steering 지연 무시 -> 나중에 고려해야 함
        double steering_angle_ = control_trajectory(i, CONTROL_SPACE::steering);

        const double x_ = state_trajectory_(i, STATE_SPACE::x);
        const double y_ = state_trajectory_(i, STATE_SPACE::y);;
        const double yaw_ = state_trajectory_(i, STATE_SPACE::yaw);
        const double velocity_ = state_trajectory_(i, STATE_SPACE::velocity);
        const double steering_ = state_trajectory_(i, STATE_SPACE::steering);

        // kinematic bicycle model
        const double sideslip_ = atan(lf_ / (lf_ + lr_) * tan(steering_angle_));
        const double delta_x_ = velocity_ * cos(yaw_ + sideslip_) * prediction_interval_;
        const double delta_y_ = velocity_ * sin(yaw_ + sideslip_) * prediction_interval_;
        const double delta_yaw_ = velocity_ * sin(sideslip_) / lr_ * prediction_interval_;
        const double delta_steering_ = steering_;

        double next_velocity_ = 0.0;
        if (1) {
            next_velocity_ = predict_constant_speed(velocity_);
        }

        // next state
        state_trajectory_(i + 1, STATE_SPACE::x) = x_ + delta_x_;
        state_trajectory_(i + 1, STATE_SPACE::y) = y_ + delta_y_;
        state_trajectory_(i + 1, STATE_SPACE::yaw) = std::atan2(sin(yaw_ + delta_yaw_), cos(yaw_ + delta_yaw_));
        state_trajectory_(i + 1, STATE_SPACE::velocity) = next_velocity_;
        state_trajectory_(i + 1, STATE_SPACE::steering) = steering_ + delta_steering_;
    }

    return state_trajectory_;
}

double SVGMPPI::predict_constant_speed(
    const double& current_speed
) const
{
    return current_speed;
}

// 하나의 state trajectory에 대한 하나의 비용을 계산한다.
std::pair<double, double> SVGMPPI::calculate_state_trajectory_cost(
    const StateMeanTrajectory state_trajectory,
    const grid_map::GridMap& local_cost_map
) const
{
    // total cost of summing state cost and terminal cost
    double collision_cost_sum_ = 0.0;

    // state cost
    for (size_t i = 0; i < prediction_step_size_; i++) {
        double collision_state_cost_ = 10.0;

        State state_ = state_trajectory.row(i);
        if (local_cost_map.isInside(grid_map::Position(state_(STATE_SPACE::x), state_(STATE_SPACE::y)))) {
            collision_state_cost_ = local_cost_map.atPosition(
                "collision", grid_map::Position(state_(STATE_SPACE::x), state_(STATE_SPACE::y))
            );
        }

        collision_cost_sum_ += collision_weight_ * collision_state_cost_;
    }

    // terminal cost
    const State terminal_state_ = state_trajectory.row(prediction_step_size_ - 1);
    double collision_terminal_cost_ = 10.0;
    if (local_cost_map.isInside(grid_map::Position(terminal_state_(STATE_SPACE::x), terminal_state_(STATE_SPACE::y)))) {
        collision_terminal_cost_ = local_cost_map.atPosition(
            "collision", grid_map::Position(terminal_state_(STATE_SPACE::x), terminal_state_(STATE_SPACE::y))
        );
    }

    collision_cost_sum_ += collision_weight_ * collision_terminal_cost_;

    return std::make_pair(collision_cost_sum_, collision_cost_sum_);
}

ControlMeanTrajectoryBatch SVGMPPI::approximate_gradient_log_posterior_batch(
    const State& current_state
)
{
    ControlMeanTrajectoryBatch gradient_log_posterior_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim)
    );

    const ControlMeanTrajectory _control_mean_trajectory_ = control_mean_trajectory_;

    for (size_t i = 0; i < sample_number_; i++) {
        const ControlMeanTrajectory gradient_log_likelihood_ = approximate_gradient_log_likelihood(
            current_state,
            _control_mean_trajectory_,
            noised_control_mean_sample_trajectory_batch_[i],
            control_inverse_covariance_trajectory_
        );

        gradient_log_posterior_batch_[i] = gradient_log_likelihood_;
    }

    return gradient_log_posterior_batch_;
}
ControlMeanTrajectory SVGMPPI::approximate_gradient_log_likelihood(
    const State& current_state,
    const ControlMeanTrajectory& mean_trajectory,
    const ControlMeanTrajectory& noised_seq,
    const ControlCovarianceTrajectory& inv_covs
)
{
    ControlCovarianceTrajectory gradient_covariance_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    );
    for (auto& gradient_covariance : gradient_covariance_) {
        for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
            gradient_covariance(i, i) = steering_control_covariance_for_gradient_estimation_[i];
        }
    }

    // generate gaussian random samples, center of which is input seq
    random_sampling(noised_seq, gradient_covariance_);

    // calculate forward simulation and consts
    auto cost_batch_ = calculate_sample_cost_batch(current_state, local_cost_map_, &state_trajectory_batch_).first;

    //calculate cost with control term
    std::vector<double> exp_costs(sample_number_);
    ControlMeanTrajectory sum_of_grads = mean_trajectory * 0.0;
    const ControlCovarianceTrajectory sampler_inverse_covariance = control_inverse_covariance_trajectory_;
    #pragma opm parallel for num_threads(thread_number_)
    for (size_t i = 0; i < sample_number_; i++) {
        double cost_with_control_term_ = cost_batch_[i];
        for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
            const double diff_control_term = grad_lambda_ * (previous_control_mean_trajectory_.row(j) - noised_control_mean_sample_trajectory_batch_[i].row(j)) * inv_covs[j] * (previous_control_mean_trajectory_.row(j) - noised_control_mean_sample_trajectory_batch_[i].row(j)).transpose();

            cost_with_control_term_ += diff_control_term;
        }
        const double exp_cost = std::exp(-cost_with_control_term_ / grad_lambda_);
        exp_costs[i] = exp_cost;

        ControlMeanTrajectory grad_log_gaussian(mean_trajectory.rows(), mean_trajectory.cols());
        grad_log_gaussian.setZero();
        for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
            grad_log_gaussian.row(j) = exp_cost * sampler_inverse_covariance[j] * (noised_control_mean_sample_trajectory_batch_[i] - noised_seq).row(j).transpose();
        }
        sum_of_grads += grad_log_gaussian;
    }

    const double sum_of_costs = std::accumulate(exp_costs.begin(), exp_costs.end(), 0.0);

    return sum_of_grads / (sum_of_costs + 1e-10);
}

void SVGMPPI::random_sampling(
    const ControlMeanTrajectory& control_mean_trajectory,
    const ControlCovarianceTrajectory& control_covariance_trajectory
)
{
    // set control mean trajectory and control covariance trajectory
    set_control_mean_trajectory(control_mean_trajectory);
    set_control_covariance_trajectory(control_covariance_trajectory);

    // set normal distributions parameters
    for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            const double standard_deviation_ = std::sqrt(control_covariance_trajectory_[i](j, j));
            std::normal_distribution<>::param_type param(0.0, standard_deviation_);
            (*normal_distribution_pointer_)[i][j].param(param);
        }
    }

    #pragma omp parallel for num_threads(thread_number_)
    for (size_t i = 0; i < sample_number_; i++) {
        // generate noise sequence
        for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
            for (size_t k = 0; k < CONTROL_SPACE::dim; k++) {
                noise_sample_trajectory_batch_[i](j, k) = (*normal_distribution_pointer_)[j][k](random_number_generators_[omp_get_thread_num()]);
            }
        }

        // sampling control trajectory with non-biased (around zero) sampling rate
        if (i < static_cast<size_t>((1 - non_biased_sampling_rate_) * sample_number_)) {
            // biased sampling
            noised_control_mean_sample_trajectory_batch_[i] = control_mean_trajectory_ + noise_sample_trajectory_batch_[i];
        } else {
            // non-biased sampling (around zero)
            noised_control_mean_sample_trajectory_batch_[i] = noise_sample_trajectory_batch_[i];
        }

        // clip input with control input constraints
        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
                noised_control_mean_sample_trajectory_batch_[i](k, j) = std::clamp(
                    noised_control_mean_sample_trajectory_batch_[i](k, j),
                    min_control_[j],
                    max_control_[j]
                );
            }
        }
    }
}

std::pair<double, double> SVGMPPI::gaussian_fitting(
    const std::vector<double>& x,
    const std::vector<double>& y
) const
{
    assert(x.size() == y.size());

    // should y is larger than 0 for log function
    std::vector<double> y_hat(y.size(), 0.0);
    std::transform(y.begin(), y.end(), y_hat.begin(), [](double y) { return std::max(y, 1e-10); });

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

    // To avoid nan;
    const double eps = 1e-5;
    const double mean = -u(1) / (2.0 * std::min(u(2), -eps));
    const double variance = std::sqrt(1.0 / (2.0 * std::abs(u(2))));

    return std::make_pair(mean, variance);
}

void SVGMPPI::test()
{   
    // test function predict_constant_speed
    double tmp = predict_constant_speed(2.0);
    // std::cout << tmp << std::endl;

    // -------------------------------------------------------------------------------------------------------------------------------

    // test function predict_state_trajectory

    State current_state_tmp_;
    std::cout << "current_state_tmp_" << std::endl;
    current_state_tmp_ << 0.0, 0.0, 0.0, 1.0, 0.1;
    std::cout << current_state_tmp_ << std::endl;

    ControlMeanTrajectory control_mean_trajectory_tmp_ = Eigen::MatrixXd::Zero(
        prediction_horizon_ - 1, CONTROL_SPACE::dim
    );
    control_mean_trajectory_tmp_ << 0.1, 0.2, 0.3;
    std::cout << "control_mean_trajectory_tmp_" << std::endl;
    std::cout << control_mean_trajectory_tmp_ << std::endl;

    StateMeanTrajectory predicted_state_trajectory_ = predict_state_trajectory(current_state_tmp_, control_mean_trajectory_tmp_);

    std::cout << "predicted_state_trajectory_\n";
    std::cout << predicted_state_trajectory_ << std::endl;

    // -------------------------------------------------------------------------------------------------------------------------------

    // // test function random_sampling
    // ControlMeanTrajectory control_mean_trajectory_tmp_ = Eigen::MatrixXd::Zero(
    //     prediction_horizon_ - 1, CONTROL_SPACE::dim
    // );
    // control_mean_trajectory_tmp_ << 0.1, 0.2, 0.3;
    // // std::cout << control_mean_trajectory_tmp_ << std::endl;

    // ControlCovarianceTrajectory control_covariance_trajectory_tmp_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
    //     prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim)
    // );
    // control_covariance_trajectory_tmp_[0] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.1;
    // control_covariance_trajectory_tmp_[1] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.2;
    // control_covariance_trajectory_tmp_[2] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * 1.3;
    // std::cout << control_covariance_trajectory_tmp_[0] << std::endl;
    // std::cout << control_covariance_trajectory_tmp_[1] << std::endl;
    // std::cout << control_covariance_trajectory_tmp_[2] << std::endl;


    // // print member variable normal_distribution_pointer_
    // std::cout << "normal_distribution_pointer_:\n";
    // for (size_t i = 0; i < normal_distribution_pointer_->size(); ++i) {
    //     for (size_t j = 0; j < CONTROL_SPACE::dim; ++j) {
    //         std::cout 
    //             << (*normal_distribution_pointer_)[i][j].mean()
    //             << " "
    //             << (*normal_distribution_pointer_)[i][j].stddev()
    //             << std::endl;
    //     }
    // }

    // // print noise_sample_trajectory_batch_
    // std::cout << "noise_sample_trajectory_batch_:\n";
    // for (size_t i = 0; i < noise_sample_trajectory_batch_.size(); ++i) {
    //     std::cout << noise_sample_trajectory_batch_[i] << "\n";
    // }

    // // print noised_control_mean_sample_trajectory_batch_
    // std::cout << "noise_sample_trajectory_batch_:\n";
    // for (size_t i = 0; i < noised_control_mean_sample_trajectory_batch_.size(); ++i) {
    //     std::cout << noised_control_mean_sample_trajectory_batch_[i] << "\n";
    // }

    // random_sampling(control_mean_trajectory_tmp_, control_covariance_trajectory_tmp_);

    // // print member variable normal_distribution_pointer_
    // std::cout << "normal_distribution_pointer_:\n";
    // for (size_t i = 0; i < normal_distribution_pointer_->size(); ++i) {
    //     for (size_t j = 0; j < CONTROL_SPACE::dim; ++j) {
    //         std::cout 
    //             << (*normal_distribution_pointer_)[i][j].mean()
    //             << " "
    //             << (*normal_distribution_pointer_)[i][j].stddev()
    //             << std::endl;
    //     }
    // }

    // // print noise_sample_trajectory_batch_
    // std::cout << "noise_sample_trajectory_batch_:\n";
    // for (size_t i = 0; i < noise_sample_trajectory_batch_.size(); ++i) {
    //     std::cout << noise_sample_trajectory_batch_[i] << "\n";
    // }

    // // print noised_control_mean_sample_trajectory_batch_
    // std::cout << "noise_sample_trajectory_batch_:\n";
    // for (size_t i = 0; i < noised_control_mean_sample_trajectory_batch_.size(); ++i) {
    //     std::cout << noised_control_mean_sample_trajectory_batch_[i] << "\n";
    // }

    // -------------------------------------------------------------------------------------------------------------------------------

    // test function approximate_gradient_log_likelihood

    // ControlCovarianceTrajectory gradient_log_likelihood_ = approximate_gradient_log_likelihood();
    // std::cout << "Printing Gradient Covariance Trajectory:" << std::endl;
    // for (size_t i = 0; i < gradient_log_likelihood_.size(); ++i) {
    //     std::cout << gradient_log_likelihood_[i] << std::endl;  // Eigen matrix 출력
    // }

    // -------------------------------------------------------------------------------------------------------------------------------
} 

} // namespace planning
} // namespace svg_mppi
