// Anthony Garcia

#pragma once

#include <random>

#include "rclcpp/rclcpp.hpp"

#include <grid_map_core/GridMap.hpp>

#include "mppi_planner/mppi_template.hpp"

namespace mppi {

namespace trajectory {

/*
 * @brief Model Path Integral Control
 */
class MPPI: public rclcpp::Node, public MPPITemplate
{
private:
    // hyper parameter
    const int iterationNumber_ = 50;
    const size_t predictionHorizon_ = 10;
    const size_t sampleNumber_ = 100;
    const size_t predictionStepSize_ = 4;
    const size_t stepSize_ = 0.05;
    const double warmStartRatio_ = 0.5;
    const double nonBiasedSamplingRate_ = 0.5;
    const std::array<double, ACTION_SPACE::dim> maxAction_ = {1.0};
    const std::array<double, ACTION_SPACE::dim> minAction_ ={-1.0};

    // local cost map object
    grid_map::GridMap localCostMap_;

    // global cost map object
    grid_map::GridMap globalCostMap_;

    ActionTrajectoryCovariance actionTrajectoryInverseCovariance_;

    ActionTrajectoryBatch actionNoiseSamples_;
    ActionTrajectoryBatch noisedActionTrajectorySamples_;

    // random noise generation using normal distribution
    std::unique_ptr<std::vector<std::array<std::normal_distribution<>, ACTION_SPACE::dim>>> normalDistributionPointer_;
    std::vector<std::mt19937> randomNumberGenerators_;
    std::discrete_distribution<> discreteDistribution_;

    StateTrajectoryBatch localStateTrajectoryBatch_;

    std::vector<double> costs_;

    // internal variables for warm start
    ActionMeanTrajectory previousActionMeanTrajectory_;
    ActionCovarianceTrajectory previousActionCovarianceTrajectory_;
    ActionMeanTrajectory previousRejectedActionMeanTrajectory_;
    ActionCovarianceTrajectory previousRejectedActionCovarianceTrajectory_;

    ActionMeanTrajectory actionMeanTrajectory_;
    ActionCovarianceTrajectory actionCovarianceTrajectory_;

public:
    MPPI();

    /**
     * @brief set local cost map
     */
    void setLocalCostMap(const grid_map::GridMap& localCostMap);

    /**
     * @brief set global cost map
     */
    void setGlobalCostMap(const grid_map::GridMap& globalCostMap);

    /**
     * @brief Solves the MPPI problem for the given state.
     * @param currentState The current state of the system.
     * @return Pair containing the optimal action trajectory and the collision rate.
     */
    std::pair<ActionMeanTrajectory, double> solve(const State& currentState);

    /**
     * @brief Generates noisy action trajectories for MPPI by adding Gaussian noise to the action mean.
     *
     * Produces action samples with both biased (around the mean) and non-biased (around zero) noise, 
     * ensuring they remain within control input limits.
     *
     * @param actionMeanTrajectory_ Mean action trajectory.
     * @param actionCovarianceTrajectory_ Covariance for each action step.
     */
    void sampleNoisedActionTrajectory(
        const ActionMeanTrajectory& actionMeanTrajectory_,
        const ActionCovarianceTrajectory& actionCovarianceTrajectory_
    );

    /**
     * @brief Computes the Probability Density Function (PDF) value for a given observed action trajectory.
     * 
     * This function calculates the PDF values based on a multivariate Gaussian distribution for each step
     * in the action trajectory. It uses the observed value, mean, and variance of each action step to 
     * compute the PDF value. The function iterates over each step in the action trajectory (up to 
     * `predictionStepSize_ - 1`), calculates the deviation between the observed and mean actions, and
     * applies the inverse square root of the variance matrix to determine the PDF value.
     * 
     * @param observedValue The observed action trajectory for which the PDF value is being computed.
     * @param mean The mean action trajectory, representing the expected actions.
     * @param variance The variance of the action trajectory at each step, stored as a list of covariance matrices.
     * @return ActionTrajectory The computed PDF values for each step in the action trajectory.
     */
    ActionTrajectory computeNormalPDFValue(
        const ActionMeanTrajectory& sample, 
        const ActionMeanTrajectory& mean, 
        const ActionCovarianceTrajectory& variance
    ) const;

    /**
     * @brief
     */
    std::vector<int> sampleWithProbability(
        const size_t& sampleNumber, 
        const std::vector<double>& probabilities
    );

    /**
     * @brief
     */
    // void shrink_copy_from(const std::vector<int>& indices);

    /**
     * @brief
     */
    std::pair<std::vector<double>, std::vector<double>> computeSampleCost(
        const State& CurrentLocalState,
        const grid_map::GridMap& localCostMap,
        StateTrajectoryBatch* localStateTrajectoryCandidates
    ) const;

    /**
     * @brief
     */
    StateTrajectory predictStateTrajectory(
        const ActionTrajectory& actionTrajectory,
        const State& currentState,
        const grid_map::GridMap& localCostMap
    ) const;

    /**
     * @brief
     */
    double predictConstantSpeed(
        const double& currentSpeed
    ) const;

    /**
     * @brief
     */
    std::pair<double, double> stateCost(
        const StateTrajectory localStateTrajectory,
        const grid_map::GridMap& localCostMap
    ) const;

    /**
     * @brief
     */
    std::pair<std::vector<double>, std::vector<double>> computeWeights() const;

    /**
     * @brief
     */
    std::vector<double> computeCostsWithControlTerm(
        const double& lambda,
        const double& alpha,
        const ActionTrajectory nominalActionTrajectory
    ) const;

    /**
     * @brief
     */
    std::pair<ActionMeanTrajectory, ActionCovarianceTrajectory> weightedMeanAndSigma(
        const std::vector<double>&
    ) const;

    /**
     * @brief
     */
    std::pair<ActionMeanTrajectory, ActionCovarianceTrajectory> updateMeanAndCovariance(
        const ActionMeanTrajectory& priorMeans,
        const ActionCovarianceTrajectory& priorCovariances,
        const ActionMeanTrajectory& weightedMeans,
        const ActionCovarianceTrajectory& weightedCovariances,
        const double stepSize
    ) const;

    /**
     * @brief
     */
    ActionMeanTrajectory interpolate(
        const ActionMeanTrajectory& actionMeanTrajectory1, 
        const ActionMeanTrajectory& actionMeanTrajectory2,
        const double& ratio
    ) const;

    /**
     * @brief
     */
    ActionCovarianceTrajectory interpolate(
        const ActionCovarianceTrajectory& actionCovarianceTrajectory1,
        const ActionCovarianceTrajectory& ActionCovarianceTrajectory2,
        const double& ratio
    ) const;


    void testComputeNormalPDFValue();

};

} // namespace trajectory

} // namespace mppi
