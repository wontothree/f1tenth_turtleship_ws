#include "mppi_planner/mppi.hpp"

namespace mppi {

namespace cpu {

MPPI::MPPI() : Node("mppi_planner_node")
{
    std::cout << "test" << std::endl;
}

/*
 * @brief solve mppi problem
 * @param current state
 * @return optimal state trajectory and optimal action trajectory
 */
std::pair<StateTrajectory, ActionTrajectory> MPPI::solve(const State& currentState)
{
    // number of collision
    int collisionNum_ = 0;

    // covariance diagonal matrix for action
    const std::array<double, ACTION_SPACE::dim> actionCovarianceDiagonal_ = {0.001, 0.001};

    // 임시
    StateTrajectory previousActionTrajectory = StateTrajectory::Zero(10, 10);
    // take previous action trajectory as current action trajectory mean
    StateTrajectory actionTrajectoryMean_ = previousActionTrajectory;

    // action trajectory covariance matrices
    ActionTrajectoryCovariance actionTrajectoryCovariance_ = mppiBasePtr_->getConstantActionTrajectoryCovariance(actionCovarianceDiagonal_); 

    // generate random noised action sequences based on previous action sequence as mean


















    StateTrajectory optimalStateTrajectory = StateTrajectory::Zero(10, 10);
    ActionTrajectory optimalActionTrajectory = ActionTrajectory::Zero(10, 10);

    return {optimalStateTrajectory, optimalActionTrajectory};
}

/*
 * @brief set local cost map
 */
void MPPI::setLocalCostMap(const grid_map::GridMap& localCostMap) { mppiBasePtr_->setLocalCostMap(localCostMap); }

/*
 * @brief set global cost map
 */
void MPPI::setGlobalCostMap(const grid_map::GridMap& globalCostMap) { mppiBasePtr_->setGlobalCostMap(globalCostMap); }

} // namespace cpu

} // namespace mppi
