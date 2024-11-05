#pragma once

#include <string>

namespace svg_mppi {

namespace STATE_SPACE {
    static constexpr int x = 0;
    static constexpr int y = 1;
    static constexpr int yaw = 2;
    static constexpr int velocity = 3;
    static constexpr int steering = 4;
    static constexpr int dim = 5;
} // STATE_SPACE

namespace CONTROL_SPACE {
    static constexpr int steering = 0;
    static constexpr int dim = 1;
} // CONTROL_SPACE

struct Parameters {
    struct Common {
        int PREDICTION_STEP_SIZE;
        double PREDICTION_INTERVAL;
        int lf;
        int lr;
        double MAX_STEERING_ANGLE;
        double MIN_STEERING_ANGLE;
    };
    Common common;

    struct SVG_MPPI {
        double LAMBDA;
        double ALPHA;
        double NON_BIASED_SAMPLING_RATE;
    };
    SVG_MPPI svg_mppi;
};

namespace planning {
    using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
    using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;

    using StateMeanTrajectory = Eigen::MatrixXd;
    using ControlMeanTrajectory = Eigen::MatrixXd;

    using ControlCovarianceTrajectory = Estd::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;

    using StateTrajectoryBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using ControlTrajectoryBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
} // namespace planning

} // namespace svg_mppi
