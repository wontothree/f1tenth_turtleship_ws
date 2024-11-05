#pragma once

#include "rclcpp/rclcpp.hpp"
#include <grid_map_core/GridMap.hpp>

#include "svg_mppi_planner/common.hpp"

namespace svg_mppi {
namespace planning {

class SVGMPPI {
private:
    const size_t sample_number_ = 100;
    const size_t prediction_step_size_ = 15;
    const double prediction_interval_ = 0.05; // s
    const double lf_ = 0.189;
    const double lr_ = 0.135;

public:
SVGMPPI();
~SVGMPPI() {};

std::pair<ControlMeanTrajectory, double> solve(const State& current_state);

std::pair<std::vector<double>, std::vector<double>> calculate_sample_cost(
    const State& current_state,
    const grid_map::GridMap& local_cost_map,
    StateTrajectoryBatch* state_trajectory_candidates
) const;

StateMeanTrajectory predict_state_trajectory(
    const ControlMeanTrajectory& control_trajectory,
    const State& current_state,
    const grid_map::GridMap& local_cost_map
) const;

};


} // namespace planning
} // namespace svg_mppi