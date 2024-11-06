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


std::pair<ControlMeanTrajectory, double> SVGMPPI::solve(const State& current_state)
{
    // transport guide particles by stein variational gradient descent
    std::vector<double> cost_history;
    std::vector<ControlMeanTrajectory> control_trajectory_history;

    for (int i = 0; i < num_svgd_iteration_; i++) {
        // transport samples by stein variational gradient descent
        // const ControlMeanTrajectoryBatch gradient_log_posterior_;
    }
}


std::pair<std::vector<double>, std::vector<double>> SVGMPPI::calculate_sample_costs(
    const State& current_state,
    const grid_map::GridMap& local_cost_map,
    StateMeanTrajectoryBatch* state_trajectory_candidates
) const
{
    std::vector<double> cost_(sample_number_);
    std::vector<double> collision_costs_(sample_number_);

    for (size_t i = 0; i < sample_number_; i++) {
        // state_trajectory_candidates->at(i); // predict_state_trajectory

        // calculate cost
        // const auto [cost_, collision_cost_]; // calculate_state_cost

        // cost_.at(i) = cost_;
        // collision_cost_.at(i) = collision_cost_;
    }

    // return std::make_pair(cost_, collision_cost_);
}

StateMeanTrajectory SVGMPPI::predict_state_trajectory(
    const ControlMeanTrajectory& control_trajectory,
    const State& current_state,
    const grid_map::GridMap& local_cost_map
) const
{
    // initializer state trajectory as 0
    StateMeanTrajectory state_trajectory_ = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);

    // set current state to state trajectory as initial state
    state_trajectory_.row(0) = current_state;

    for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
        // steering 지연 무시 -> 나중에 고려해야 함
        double steering_angle_ = 0.0;
        steering_angle_ = control_trajectory(i, CONTROL_SPACE::steering);

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
std::pair<double, double> SVGMPPI::calculate_state_cost(
    const StateMeanTrajectory state_trajectory,
    const grid_map::GridMap& local_cost_map
) const
{
    // total cost of summing state cost and terminal cost
    double collision_cost_sum_ = 0.0;

    // state cost
    for (size_t i = 0; i < prediction_step_size_; i++) {
        double collision_state_cost_ = 0.0;

        State state_ = state_trajectory.row(i);
        if (local_cost_map.isInside(grid_map::Position(state_(STATE_SPACE::x), state_(STATE_SPACE::y)))) {
            collision_state_cost_ = local_cost_map.atPosition("collision", grid_map::Position(state_(STATE_SPACE::x), state_(STATE_SPACE::y)));
        }

        collision_cost_sum_ += collision_weight_ * collision_state_cost_;
    }

    // terminal cost
    const State terminal_state_ = state_trajectory.row(prediction_step_size_ - 1);
    double collision_terminal_cost_ = 0.0;
    if (local_cost_map.isInside(grid_map::Position(terminal_state_(STATE_SPACE::x), terminal_state_(STATE_SPACE::y)))) {
        collision_terminal_cost_ = local_cost_map.atPosition("collision", grid_map::Position(terminal_state_(STATE_SPACE::x), terminal_state_(STATE_SPACE::y)));
    }

    collision_cost_sum_ += collision_weight_ * collision_terminal_cost_;

    return std::make_pair(collision_cost_sum_, collision_cost_sum_);
}

ControlMeanTrajectoryBatch SVGMPPI::approximate_gradient_posterior_batch() const
{
    ControlMeanTrajectoryBatch gradient_log_posterior_batch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sample_number_, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim) // 10은 임시
    );

    // for (size_t i = 0; i < sample_number_; i++) {
    //     const ControlMeanTrajectory gradient_log_likelihood_ = approximate_gradient_log_likelihood(

    //     )
    //     gradient_log_posterior_batch_[i] = gradient_log_likelihood_;
    // }

    return gradient_log_posterior_batch_;
}
ControlMeanTrajectory SVGMPPI::approximate_gradient_log_likelihood() const
{
    // const ControlCovarianceTrajectory 

    
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

        // add noise to mean
        // sampling control trajectory with non-biased (around zero) sampling rate
        if (i < static_cast<size_t>((1 - non_biased_sampling_rate_) * sample_number)) {
            // biased sampling
            noised_control_mean_sample_trajectory_batch_[i] = control_mean_trajectory_ + noise_sample_trajectory_batch_[i];
        } else {
            // non-biased sampling (around zero)
            noised_control_mean_sample_trajectory_batch_[i] = noise_sample_trajectory_batch_[i];
        }

        // 여기부터 할 차례 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // clip input with control input constraints
        for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
            for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
                // noised_control_mean_sample_trajectory_batch_[i](k, j) = std::clamp(noised_control_mean_sample_trajectory_batch_[i](k, j));
            }
        }
    }


}

void SVGMPPI::test()
{   
    // test function predict_constant_speed
    double tmp = predict_constant_speed(2.0);
    // std::cout << tmp << std::endl;

    // -------------------------------------------------------------------------------------------------------------------------------

    // test function predict_state_trajectory
    State current_state;
    current_state << 0.0, 0.0, 0.0, 1.0, 0.1;
    ControlMeanTrajectory control_trajectory = Eigen::MatrixXd::Zero(prediction_step_size_, CONTROL_SPACE::dim);
    // for (size_t i = 0; i < prediction_step_size_; i++) {
    //     control_trajectory(i, STATE_SPACE::steering) = 0.1;
    // }
    grid_map::GridMap local_cost_map;

    StateMeanTrajectory predicted_trajectory = predict_state_trajectory(control_trajectory, current_state, local_cost_map);

    // std::cout << "Predicted State Trajectory:\n";
    // std::cout << predicted_trajectory << std::endl;

    // -------------------------------------------------------------------------------------------------------------------------------

    // test variable normal_distribution_pointer_

    // 정규 분포의 평균과 표준편차 출력
    // for (size_t i = 0; i < normal_distribution_pointer_->size(); ++i) {
    //     std::cout << "정규 분포 집합 " << i + 1 << ":" << std::endl;
    //     for (size_t j = 0; j < CONTROL_SPACE::dim; ++j) {
    //         std::cout << "  차원 " << j + 1 << " - 평균: "
    //                     << (*normal_distribution_pointer_)[i][j].mean()
    //                     << ", 표준편차: " << (*normal_distribution_pointer_)[i][j].stddev()
    //                     << std::endl;
    //     }
    // }

    // -------------------------------------------------------------------------------------------------------------------------------

    // test function random_sampling

    // Eigen::MatrixXd 형태로 ControlMeanTrajectory 초기화
    planning::ControlMeanTrajectory control_mean_trajectory(3, 1);  // 3x1 크기 행렬
    control_mean_trajectory << 0.5, 0.7, 0.9;  // 예시 제어 평균 값 (3x1 행렬)

    // Eigen::MatrixXd 형태로 ControlCovarianceTrajectory 초기화
    planning::ControlCovarianceTrajectory control_covariance_trajectory;
    control_covariance_trajectory.push_back(Eigen::MatrixXd::Identity(1, 1) * 0.1);  // 1x1 공분산 행렬
    control_covariance_trajectory.push_back(Eigen::MatrixXd::Identity(1, 1) * 0.2);  // 1x1 공분산 행렬
    control_covariance_trajectory.push_back(Eigen::MatrixXd::Identity(1, 1) * 0.3);  // 1x1 공분산 행렬 


    // // 제어 평균 출력
    // std::cout << "Control Mean Trajectory:" << std::endl;
    // std::cout << control_mean_trajectory << std::endl;

    // // 제어 공분산 출력
    // std::cout << "\nControl Covariance Trajectory:" << std::endl;
    // for (size_t i = 0; i < control_covariance_trajectory.size(); ++i) {
    //     std::cout << "Covariance at time " << i + 1 << ":\n" << control_covariance_trajectory[i] << std::endl;
    // }

    random_sampling(control_mean_trajectory, control_covariance_trajectory);

    // for (size_t i = 0; i < normal_distribution_pointer_->size(); i++) {
    //     std::cout << "Normal Distribution Set " << i + 1 << ":" << std::endl;
    //     for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
    //         const auto& dist = (*normal_distribution_pointer_)[i][j];
    //         std::cout << "  Dimension " << j + 1 << " - Mean: " << dist.mean() 
    //                   << ", Std Dev: " << dist.stddev() << std::endl;
    //     }
    // }

    std::cout << "Noise Sample Trajectory Batch:" << std::endl;
    for (size_t i = 0; i < sample_number_; i++) {
        std::cout << "Sample " << i + 1 << ":" << std::endl;
        for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
            for (size_t k = 0; k < CONTROL_SPACE::dim; k++) {
                std::cout << noise_sample_trajectory_batch_[i](j, k) << " ";
            }
            std::cout << std::endl;  // 각 행을 출력 후 줄 바꿈
        }
        std::cout << "------------------------------" << std::endl;
    }

    // -------------------------------------------------------------------------------------------------------------------------------

    // test function calculate_state_cost


} 

} // namespace planning
} // namespace svg_mppi
