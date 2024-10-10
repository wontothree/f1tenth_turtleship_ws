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

// namespace mppi_types {
//     using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
//     using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;
    
//     using StateSeq = Eigen::MatrixXd;
//     using ControlSeq = Eigen::MatrixXd;
// } // mpc_types

} // mppi