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
    const double collision_weight_ = 1.0;
    const size_t num_svgd_iteration_ = 3;

public:
SVGMPPI();
~SVGMPPI() {};

std::pair<ControlMeanTrajectory, double> solve(const State& current_state);

std::pair<std::vector<double>, std::vector<double>> calculate_sample_costs(
    const State& current_state,
    const grid_map::GridMap& local_cost_map,
    StateMeanTrajectoryBatch* state_trajectory_candidates
) const;

StateMeanTrajectory predict_state_trajectory(
    const ControlMeanTrajectory& control_trajectory,
    const State& current_state,
    const grid_map::GridMap& local_cost_map
) const;

double predict_constant_speed(
    const double& current_speed
) const;

std::pair<double, double> calculate_state_cost(
    const StateMeanTrajectory state_trajectory,
    const grid_map::GridMap& local_cost_map
) const;



void test();

};

} // namespace planning
} // namespace svg_mppi