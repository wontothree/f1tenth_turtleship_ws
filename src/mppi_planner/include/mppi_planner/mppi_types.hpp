// Anthony Garcia

#pragma once

#include <string>

namespace mppi {

namespace STATE_SPACE {
    static constexpr int x = 0;
    static constexpr int y = 1;
    static constexpr int yaw = 2;
    static constexpr int vel = 3;
    static constexpr int steer = 4;
    static constexpr int dim = 5;
} // STATE_SPACE

namespace ACTION_SPACE {
    static constexpr int steer = 0;
    static constexpr int accel = 1;
    static constexpr int dim = 2;
} // ACTION_SPACE

namespace cpu {
    using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
    using Action = Eigen::Matrix<double, ACTION_SPACE::dim, 1>;
    
    using StateTrajectory = Eigen::MatrixXd;
    using ActionTrajectory = Eigen::MatrixXd;

    using ActionTrajectoryCovariance = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;

    // multiple trajectory for state and action
    using StateTrajectoryBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using ActionTrajectoryBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
} // mpc_types

} // mppi
