#pragma once

#include <string>

namespace mppi {

namespace STATE_SPACE {
    static constexpr int x = 0;     // m
    static constexpr int y = 1;     // m
    static constexpr int yaw = 2;   // rad
    static constexpr int vel = 3;   // m/s
    static constexpr int steer = 4; // rad
    static constexpr int dim = 5;
} // STATE_SPACE

namespace ACTION_SPACE {
    static constexpr int steer = 0;
    static constexpr int dim = 1;
} // ACTION_SPACE

struct Parameters {
    struct Common {
        int THREAD_NUMBERS;
        int PREDICTION_STEP_SIZE;   // scalar
        double PREDICTION_INTERVAL; // s
        double MAX_STEER_ANGLE;
        double MIN_STEER_ANGLE;
        double lf;
        double lr;
    };
    Common common;

    struct MPPI {
        int iterationNumber;
        double lambda;
    };
    MPPI mppi;
};

namespace trajectory {
    using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
    using Action = Eigen::Matrix<double, ACTION_SPACE::dim, 1>;
    using StateTrajectory = Eigen::MatrixXd;
    using ActionTrajectory = Eigen::MatrixXd;
    using StateMeanTrajectory = Eigen::MatrixXd;
    using ActionMeanTrajectory = Eigen::MatrixXd;
    using ActionTrajectoryCovariance = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using ActionCovarianceTrajectory = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using StateTrajectoryBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using ActionTrajectoryBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
} // namespace trajectory

} // namespace mppi
