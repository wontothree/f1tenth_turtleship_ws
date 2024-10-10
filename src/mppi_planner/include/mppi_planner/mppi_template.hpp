// Anthony Garcia

#pragma once

#include <utility>
#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>

#include "local_planner/mpc_type.hpp"

class MPCTemplate 
{
public:
    virtual ~MPCTemplate() = default;

    virtual std::pair<ControlSeq, double> solve(const State& initial_state) = 0;

    /* 
     * setter function for local costmap
     */
    virtual void set_local_costmap(const grid_map::GridMap& local_costmap) = 0;

    /*
     * setter function for global costmap
     */
    virtual void set_global_costmap(const grid_map::GridMap& global_costmap) = 0;

    virtual ControlSeq get_control_seq() const = 0;

    virtual std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq(const int& num_samples) const = 0;

    // virtual std::tuple<StateSeq, double, double, double> get_predictive_seq(const State& initial_state, const ControlSeq& control_input_seq) const = 0;

    // virtual ControlSeqCovMatrices get_cov_matrices() const = 0;

    // virtual std::pair<StateSeq, XYCovMatrices> get_proposed_state_distribution() const = 0;

    // std::vector<double> softmax(const std::vector<double>& costs, const double& lambda, const in thread_num) const
    // {
    //     const double min_cost = *std::min_element(costs.begin(), costs.end());
    //     double normalization_term = 1e-10;

    //     for (const auto& cost : costs) {
    //         normalization_term += std::exp(-(cost - min_cost) / lambda);
    //     }

    //     std::vector<double> softmax_costs(costs.size());

    //     #pragma omp parallel for num_threads(thread_num)
    //     for (size_t i = 0; i < costs.size(); i++) {
    //         softmax_costs[i] = std::exp(-(costs[i] - min_cost) / lambda) / normalization_term;
    //     }

    //     return softmax_costs;
    // }

private:
}