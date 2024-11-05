#include "mppi_planner/mppi.hpp"

namespace mppi {

namespace trajectory {

MPPI::MPPI() : Node("mppi_planner_node")
{
    // testComputeNormalPDFValue();

    actionTrajectoryInverseCovariance_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionHorizon_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim)
    );

    actionNoiseSamples_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sampleNumber_, Eigen::MatrixXd::Zero(predictionHorizon_ - 1, ACTION_SPACE::dim)
    );

    noisedActionTrajectorySamples_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sampleNumber_, Eigen::MatrixXd::Zero(predictionHorizon_ - 1, ACTION_SPACE::dim)
    );

    costs_ = std::vector<double>(sampleNumber_, 0.0);

    // initialize normal distribution for random noise
    normalDistributionPointer_ = std::make_unique<std::vector<std::array<std::normal_distribution<>, ACTION_SPACE::dim>>>();
    for (size_t i = 0; i < predictionHorizon_ - 1; i++) {

        // ACTION_SPACE::dim 개의 normal distribution 객체 생성과 표준정규분포를 이용한 초기화
        std::array<std::normal_distribution<>, ACTION_SPACE::dim> normalDistributions_ = {};
        for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
            // 표준정규분포
            std::normal_distribution<> standardNormalDistribution_(0.0, 1.0);

            // 초기화
            normalDistributions_[j] = standardNormalDistribution_;
        }
        // 초기화
        (*normalDistributionPointer_).push_back(normalDistributions_);
    }

    // initialize random number generator
    randomNumberGenerators_.resize(predictionHorizon_);
    unsigned int seed = 123456789;  // 기본 시드 값
    for (size_t i = 0; i < predictionHorizon_; ++i) {
        randomNumberGenerators_[i] = std::mt19937(seed + i);  // 각 엔진에 다른 시드로 초기화
    }

    localStateTrajectoryBatch_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        sampleNumber_, Eigen::MatrixXd::Zero(predictionStepSize_, STATE_SPACE::dim)
    );


    // internal variables for warm start
    const std::array<double, ACTION_SPACE::dim> actionCovarianceDiagonal_ = {0.1};

    previousActionMeanTrajectory_ = Eigen::MatrixXd::Zero(predictionHorizon_ - 1, ACTION_SPACE::dim);

    previousActionCovarianceTrajectory_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionHorizon_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim)
    );

    for (auto& cov : previousActionCovarianceTrajectory_) {
        for (size_t i = 0; i < ACTION_SPACE::dim; i++) {
            cov(i, i) = actionCovarianceDiagonal_[i];
        }
    }

    previousRejectedActionMeanTrajectory_ = Eigen::MatrixXd::Zero(predictionHorizon_ - 1, ACTION_SPACE::dim);

    previousRejectedActionCovarianceTrajectory_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionHorizon_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim)
    );

    for (auto& cov : previousRejectedActionCovarianceTrajectory_) {
        for (size_t i = 0; i < ACTION_SPACE::dim; i++) {
            cov(i, i) = actionCovarianceDiagonal_[i];
        }
    }

    // action trajectory
    actionMeanTrajectory_ = Eigen::MatrixXd::Zero(predictionHorizon_ - 1, ACTION_SPACE::dim);

    actionCovarianceTrajectory_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionHorizon_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim)
    );

    sampleNoisedActionTrajectory(actionMeanTrajectory_, actionCovarianceTrajectory_);
}

/*
 * @brief set local cost map
 */
void MPPI::setLocalCostMap(const grid_map::GridMap& localCostMap) { localCostMap_ = localCostMap; }

/*
 * @brief set global cost map
 */
void MPPI::setGlobalCostMap(const grid_map::GridMap& globalCostMap) { globalCostMap_ = globalCostMap; }

/**
 * @brief Solves the MPPI problem for the given state.
 * @param currentState The current state of the system.
 * @return Pair containing the optimal action trajectory and the collision rate.
 */
std::pair<ActionMeanTrajectory, double> MPPI::solve(
    const State& currentState
)
{
    // 전체 샘플 중에서 충돌이 발생한 샘플의 개수
    int collisionCount_ = 0;
    
    // warm start
    ActionMeanTrajectory actionMeanTrajectory_ = previousActionMeanTrajectory_;
    ActionCovarianceTrajectory actionCovarianceTrajectory_ = previousActionCovarianceTrajectory_;
    ActionMeanTrajectory rejectedActionMeanTrajectory_ = previousRejectedActionMeanTrajectory_;
    ActionCovarianceTrajectory rejectedActionCovarianceTrajectory_ = previousRejectedActionCovarianceTrajectory_;
    for (int i = 0; i < iterationNumber_; i++)
    {
        // sampling
        sampleNoisedActionTrajectory(actionMeanTrajectory_, actionCovarianceTrajectory_);

        // calculate sample rejection probabilities
        ActionTrajectory rejectedActionMeanTrajectoryNormalPDFValue_ = computeNormalPDFValue(rejectedActionMeanTrajectory_, actionMeanTrajectory_, actionCovarianceTrajectory_);

        // probabilities vector
        std::vector<double> probabilities(sampleNumber_, 0.0);
        for (size_t j = 0; j < sampleNumber_; j++) {
            const ActionTrajectory pr = computeNormalPDFValue(noisedActionTrajectorySamples_[j], rejectedActionMeanTrajectory_, rejectedActionCovarianceTrajectory_);
            probabilities[j] = (rejectedActionMeanTrajectoryNormalPDFValue_ + pr).cwiseInverse().prod();
        }

        // normalize probabilities vector
        const double probabilitesSum_ = std::min(std::accumulate(probabilities.begin(), probabilities.end(), 0.0), 1e-10);
        for (size_t j = 0; j < 100; j++) {
            probabilities[j] /= probabilitesSum_;
        }

        const std::vector<int> selectedIndices = sampleWithProbability(sampleNumber_, probabilities);
        // shrink_copy_from(selectedIndices);

        // predict and calculate trajectory costs
        auto [costs_, collisionCosts_] = computeSampleCost(currentState, localCostMap_, &localStateTrajectoryBatch_);
        collisionCount_ = std::count_if(collisionCosts_.begin(), collisionCosts_.end(), [](const double& cost) { return cost > 0.0; });

        // calculate weights for each state sequence based on costs and prior samples
        const auto [positiveSampleWeights_, negativeSampleWeights_] = computeWeights();

        // calculate weighted mean and standard deviation of evaluated and rejected samples
        const auto [mean_eval, sigma_eval] = weightedMeanAndSigma(positiveSampleWeights_);
        const auto [mean_reject, sigma_reject] = weightedMeanAndSigma(negativeSampleWeights_);

        // MD update
        const auto [updateMean_, updateCovariance_] = updateMeanAndCovariance(actionMeanTrajectory_, actionCovarianceTrajectory_, mean_eval, sigma_eval, stepSize_);
        actionMeanTrajectory_ = updateMean_;
        actionCovarianceTrajectory_ = updateCovariance_;

        const auto [updatedRejectedMean_, updatedRejectedCovariance_] = updateMeanAndCovariance(rejectedActionMeanTrajectory_, rejectedActionCovarianceTrajectory_, mean_reject, sigma_reject, stepSize_);
        rejectedActionMeanTrajectory_ = updatedRejectedMean_;
        rejectedActionCovarianceTrajectory_ = updatedRejectedCovariance_;
    }

    // Warm start for next control iteration
    previousActionMeanTrajectory_ = interpolate(previousActionMeanTrajectory_, actionMeanTrajectory_, warmStartRatio_);
    previousActionCovarianceTrajectory_ = interpolate(previousActionCovarianceTrajectory_, actionCovarianceTrajectory_, warmStartRatio_);
    previousRejectedActionMeanTrajectory_ = interpolate(previousRejectedActionMeanTrajectory_, rejectedActionMeanTrajectory_, warmStartRatio_); 
    previousRejectedActionCovarianceTrajectory_ = interpolate(previousRejectedActionCovarianceTrajectory_, rejectedActionCovarianceTrajectory_, warmStartRatio_);

    const double collisionRate_ = static_cast<double>(collisionCount_) / static_cast<double>(sampleNumber_);

    return std::make_pair(previousActionMeanTrajectory_, collisionRate_);
}

/**
 * @brief Generates noisy action trajectories for MPPI by adding Gaussian noise to the action mean.
 *
 * Produces action samples with both biased (around the mean) and non-biased (around zero) noise, 
 * ensuring they remain within control input limits.
 *
 * @param actionMeanTrajectory Mean action trajectory.
 * @param actionCovarianceTrajectory Covariance for each action step.
 */
void MPPI::sampleNoisedActionTrajectory(
    const ActionMeanTrajectory& actionMeanTrajectory,
    const ActionCovarianceTrajectory& actionCovarianceTrajectory
)
{
    // set normal distributions parameters
    for (size_t i = 0; i < predictionHorizon_ - 1; i++) {
        for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
            // standard deviation in each time step
            const double standardDeviation_ = std::sqrt(actionCovarianceTrajectory_[i](j, j));

            // normal distribution for noise
            std::normal_distribution<>::param_type noiseNormalDistribution_(0.0, standardDeviation_);

            //
            (*normalDistributionPointer_)[i][j].param(noiseNormalDistribution_);
        }
    }

    // // 
    // for (size_t i = 0; i < sampleNumber_; i++) {
    //     // generate noise sequence
    //     for (size_t j = 0; j < predictionHorizon_ - 1; j++) {
    //         for (size_t k = 0; k < ACTION_SPACE::dim; k++) {
    //             actionNoiseSamples_[i](j, k) = (*normalDistributionPointer_)[j][k](randomNumberGenerators_[1]); // v
    //         }
    //     }
    //     std::cout << actionNoiseSamples_[i] << std::endl;

    //     // // sampling action sequences with non-biased (around zero) sampling rate
    //     // if (i < static_cast<size_t>((1 - nonBiasedSamplingRate_) * sampleNumber_)) {
    //     //     // biased sampling (around actionMeanTrajectory_)
    //     //     noisedActionTrajectorySamples_[i] = actionMeanTrajectory + actionNoiseSamples_[i];
    //     // } else {
    //     //     // non-biased sampling (around zero)
    //     //     noisedActionTrajectorySamples_[i] = actionNoiseSamples_[i];
    //     // }

    //     // // clip input with control input constraints
    //     // for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
    //     //     for (size_t k = 0; k < predictionHorizon_ - 1; k++) {
    //     //         noisedActionTrajectorySamples_[i](k, j) = std::clamp(noisedActionTrajectorySamples_[i](k, j), minAction_[j], maxAction_[j]);
    //     //     }
    //     // }
    // }
    // // 생성된 노이즈가 추가된 액션 경로 출력
    // for (size_t i = 0; i < sampleNumber_; i++) {
    //     std::cout << "Sample " << i << " Noised Action Trajectory:" << std::endl;
    //     for (size_t j = 0; j < predictionHorizon_ - 1; j++) {
    //         std::cout << "Time step " << j << ": ";
    //         for (size_t k = 0; k < ACTION_SPACE::dim; k++) {
    //             std::cout << noisedActionTrajectorySamples_[i](j, k) << " ";
    //         }
    //         std::cout << std::endl;
    //     }
    //     std::cout << std::endl; // 각 샘플 간의 간격 추가
    // }
}

ActionTrajectory MPPI::computeNormalPDFValue(
    const ActionMeanTrajectory& sample,
    const ActionMeanTrajectory& mean,
    const ActionCovarianceTrajectory& variance
) const {
    // initialize matrix as 0
    ActionMeanTrajectory normalPDFValue = ActionMeanTrajectory::Zero(sample.rows(), sample.cols());

    for (size_t i = 0; i < 4 - 1; i++) {
        normalPDFValue.row(i) = (sample.row(i) - mean.row(i))
                            .cwiseProduct((variance[i].diagonal().cwiseSqrt().asDiagonal().inverse() * (sample.row(i) - mean.row(i)).transpose()).transpose())
                            .array()
                            .exp() /
                        std::sqrt(std::pow(2.0 * M_PI, ACTION_SPACE::dim) * variance[i].determinant());
    }

    return normalPDFValue;
}

std::vector<int> MPPI::sampleWithProbability(
    const size_t& sampleNumber,
    const std::vector<double>& probabilities
)
{   
    // inspect validity
    if (probabilities.size() != sampleNumber) {
        std::cout << "Error: probability size is not equal to the number of samples." << std::endl;
        exit(1);
    }

    // choose random indices with probability
    discreteDistribution_.param(std::discrete_distribution<>::param_type(probabilities.begin(), probabilities.end()));
    std::vector<int> indices(sampleNumber);
    for (size_t i = 0; i < sampleNumber; i++) {
        indices[i] = discreteDistribution_(randomNumberGenerators_[i % randomNumberGenerators_.size()]);
    }

    return indices;
}

/**
 * @brief
 */
// void MPPI::shrink_copy_from(
//     const std::vector<int>& indices
// )
// {
//     // inspect validity
//     if (indices.size() != num_samples_) {
//         std::cout << "Error: indices size is not equal to num_samples_." << std::endl;
//         exit(1);
//     }
//     if (source_samples.prediction_horizon_ != prediction_horizon_) {
//         std::cout << "Error: source_samples.prediction_horizon_ is not equal to prediction_horizon_." << std::endl;
//         exit(1);
//     }

//     if (source_samples.num_samples_ < indices.size()) {
//         std::cout << "Error: source_samples.num_samples_ is smaller than indices.size()." << std::endl;
//         exit(1);
//     }

//     for (size_t i = 0; i < indices.size(); i++) {
//         noised_control_seq_samples_[i] = source_samples.noised_control_seq_samples_[indices[i]];
//         noise_seq_samples_[i] = source_samples.noise_seq_samples_[indices[i]];
//         costs_[i] = source_samples.costs_[indices[i]];
//     }
//     set_control_seq_mean(source_samples.control_seq_mean_);
//     set_control_seq_cov_matrices(source_samples.control_seq_cov_matrices_);
// }

// }

/**
 * @brief
 */
std::pair<std::vector<double>, std::vector<double>> MPPI::computeSampleCost(
    const State& currentLocalState,
    const grid_map::GridMap& localCostMap,
    StateTrajectoryBatch* localStateTrajectoryCandidates
) const
{
    std::vector<double> costs_(100); // number of samples
    std::vector<double> collisionCosts_(100);

    // Iterate over each control sequence to simulate its effect on the system
    // and evaluate the resulting state trajectories and associated costs.
    for (size_t i = 0; i < 100; i++) {
        // predict state trajectory
        localStateTrajectoryCandidates->at(i) = predictStateTrajectory(noisedActionTrajectorySamples_[i], currentLocalState, localCostMap);

        // calculate cost
        const auto [cost_ ,collisionCost_] = stateCost(localStateTrajectoryCandidates->at(i), localCostMap);
        costs_.at(i) = cost_;
        collisionCosts_.at(i) = collisionCost_;
    }

    return std::make_pair(costs_, collisionCosts_);
}

/**
 * @brief
 */
StateTrajectory MPPI::predictStateTrajectory(
    const ActionTrajectory& actionTrajectory,
    const State& currentState,
    const grid_map::GridMap& localCostMap
) const
{
    // initialize as 0
    StateTrajectory stateTrajectory_ = Eigen::MatrixXd::Zero(predictionStepSize_, STATE_SPACE::dim);

    // set current state to state trajectory
    stateTrajectory_.row(0) = currentState;

    for (size_t i = 0; i < predictionStepSize_ - 1; i++) {
        // steering 지연 고려

        const double x = stateTrajectory_(i, STATE_SPACE::x);
        const double y = stateTrajectory_(i, STATE_SPACE::y);
        const double yaw = stateTrajectory_(i, STATE_SPACE::yaw);
        const double vel = stateTrajectory_(i, STATE_SPACE::vel);
        const double steer = stateTrajectory_(i, STATE_SPACE::steer);

        // bicycle model kinematics
        const double beta = 0.0; // = atan(lf_ / (lf_ + lr_) * tan(steer_angle));
        const double deltaX = 0.0; // = vel * cos(yaw + beta) * prediction_interval_;
        const double deltaY = 0.0; // = vel * sin(yaw + beta) * prediction_interval_;
        const double deltaYaw = 0.0; // = vel * sin(beta) / lr_ * prediction_interval_;
        const double deltaSteer = 0.0; // = ((steer_angle - steer) / steer_delay_tau_) * prediction_interval_;

        double nextVel_ = 0.0;
        if (1) {
            nextVel_ = predictConstantSpeed(vel);
        } // speed prediction mode

        // next state from kinematics
        stateTrajectory_(i + 1, STATE_SPACE::x) = x + deltaX;
        stateTrajectory_(i + 1, STATE_SPACE::y) = y + deltaY;
        stateTrajectory_(i + 1, STATE_SPACE::yaw) = std::atan2(sin(yaw + deltaYaw), cos(yaw + deltaYaw));
        stateTrajectory_(i + 1, STATE_SPACE::vel) = nextVel_;
        stateTrajectory_(i + 1, STATE_SPACE::steer) = steer + deltaSteer;
    }

    return stateTrajectory_;
}

/**
 * @brief
 */
double MPPI::predictConstantSpeed(
    const double& currentSpeed
) const
{
    return currentSpeed;
}

/**
 * @brief 
 */
std::pair<double, double> MPPI::stateCost(
    const StateTrajectory localStateTrajectory,
    const grid_map::GridMap& localCostMap
) const
{
    // calculate cost for each state

    // state cost
    double collisionCostSum_ = 0.0;
    for (size_t i = 0; i < predictionStepSize_ - 1; i++) {
        const State localState_ = localStateTrajectory.row(i);

        // collision cost
        double collisionCost_ = 10.0;
        if (localCostMap.isInside(grid_map::Position(localState_(STATE_SPACE::x), localState_(STATE_SPACE::y)))) {
            collisionCost_ = localCostMap.atPosition("collision_layer", grid_map::Position(localState_(STATE_SPACE::x), localState_(STATE_SPACE::y)));
        }

        collisionCostSum_ += collisionCost_ * 10; // collision weight
    }

    // terminal cost
    const State terminalLocalState_ = localStateTrajectory.row(predictionStepSize_ - 1);
    double collisionCost_ = 10.0;
    if (localCostMap.isInside(grid_map::Position(terminalLocalState_(STATE_SPACE::x), terminalLocalState_(STATE_SPACE::y)))) {
        collisionCost_ = localCostMap.atPosition("collision_layer", grid_map::Position(terminalLocalState_(STATE_SPACE::x), terminalLocalState_(STATE_SPACE::y)));
    }

    collisionCostSum_ += collisionCost_ * 10;

    return std::make_pair(collisionCostSum_, collisionCostSum_);
}

std::pair<std::vector<double>, std::vector<double>> MPPI::computeWeights() const
{
    ActionTrajectory zeroControlTrajectory_ = Eigen::MatrixXd::Zero(predictionHorizon_, STATE_SPACE::dim);
    // calculate costs with control cost term
    const std::vector<double> costsWithControlTerm_ = computeCostsWithControlTerm(3.0, 0.1, zeroControlTrajectory_); 

    // calculate normalization term
    double positiveNormalizationTerm_ = 1e-10;
    double negativeNormalizationTerm_ = 1e-10;
    const double minCost_ = *std::min_element(costsWithControlTerm_.begin(), costsWithControlTerm_.end());
    for (size_t i = 0; i < sampleNumber_; i++) {
        positiveNormalizationTerm_ += std::exp(-1.0 / 3.0 * (costsWithControlTerm_[i] - minCost_));
        negativeNormalizationTerm_ += std::exp( 1.0 / 3.0 * (costsWithControlTerm_[i] - minCost_));
    }

    std::vector<double> positiveSampleWeights_(sampleNumber_, 0.0);
    std::vector<double> negativeSampleWeights_(sampleNumber_, 0.0);

    // calculate weights for importance sampling
    for (size_t i = 0; i < sampleNumber_; i++) {
        const double positiveTerm_ = std::exp(-1.0 / 3.0 * (costsWithControlTerm_[i] - minCost_)) / positiveNormalizationTerm_;
        const double negativeTerm_ = 1.0 * std::exp(1.0 / 3.0 * ( costsWithControlTerm_[i] - minCost_)) / negativeNormalizationTerm_;

        const double reversedWeight_ = positiveTerm_ - negativeTerm_;
        positiveSampleWeights_[i] = std::max(reversedWeight_, 0.0);
        negativeSampleWeights_[i] = -std::min(reversedWeight_, 0.0);
    }

    // normalize weights
    const double positiveSampleWeightsSum_ = std::accumulate(positiveSampleWeights_.begin(), positiveSampleWeights_.end(), 0.0);
    const double negativeSampleWeightsSum_ = std::accumulate(negativeSampleWeights_.begin(), negativeSampleWeights_.end(), 0.0);
    for (size_t i = 0; i < sampleNumber_; i++) {
        positiveSampleWeights_[i] /= positiveSampleWeightsSum_;
        negativeSampleWeights_[i] /= negativeSampleWeightsSum_;
    }

    return std::make_pair(positiveSampleWeights_, negativeSampleWeights_);
}

std::vector<double> MPPI::computeCostsWithControlTerm(
    const double& lambda,
    const double& alpha,
    const ActionTrajectory nominalActionTrajectory
) const
{
    std::vector<double> costsWithControlTerm_ = costs_;
    for (size_t i = 0; i < sampleNumber_; i++) {
        for (size_t j = 0; j < predictionHorizon_ -1; j++) {
            const double controlTerm_ = lambda * (1 - alpha) * (actionMeanTrajectory_.row(j) - nominalActionTrajectory.row(j)) * actionTrajectoryInverseCovariance_[j] * noisedActionTrajectorySamples_[i].row(j).transpose();
            costsWithControlTerm_[i] += controlTerm_;
        }
    }

    return costsWithControlTerm_;
}

std::pair<ActionMeanTrajectory, ActionCovarianceTrajectory> MPPI::weightedMeanAndSigma(
    const std::vector<double>& weights
) const
{
    ActionMeanTrajectory mean_ = Eigen::MatrixXd::Zero(predictionHorizon_ - 1, ACTION_SPACE::dim);
    ActionCovarianceTrajectory sigma_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(predictionHorizon_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim));

    const ActionMeanTrajectory priorMean_ = actionMeanTrajectory_;
    const ActionCovarianceTrajectory priorInverseCovariance_ = actionTrajectoryInverseCovariance_;

    for (size_t i = 0; i < sampleNumber_; i++) {
        mean_ += weights[i] * noisedActionTrajectorySamples_[i];

        const ActionTrajectory difference = noisedActionTrajectorySamples_[i] - priorMean_;

        for (size_t j = 0; j < predictionStepSize_ - 1; j++) {
            sigma_[j] += weights[i] * difference.row(j).transpose() * priorInverseCovariance_[j] * difference.row(j);
        }
    }

    return std::make_pair(mean_, sigma_);
}

std::pair<ActionMeanTrajectory, ActionCovarianceTrajectory> MPPI::updateMeanAndCovariance(
    const ActionMeanTrajectory& priorMean,
    const ActionCovarianceTrajectory& priorCovariance,
    const ActionMeanTrajectory& weightedMean,
    const ActionCovarianceTrajectory& weightedCovariance,
    const double stepSize
) const
{
    // update mean
    ActionMeanTrajectory updateMean_ = priorMean * 0.0;
    updateMean_ = priorMean - stepSize * (priorMean - weightedMean);

    // update covariance
    ActionCovarianceTrajectory priorStandardDeviations_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionStepSize_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim)
    );
    ActionCovarianceTrajectory weightedStandardDeviations_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionStepSize_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim)
    );
    for (size_t i = 0; i < predictionStepSize_ - 1; i++) {
        priorStandardDeviations_[i] = priorCovariance[i].diagonal().cwiseSqrt().asDiagonal();
        weightedStandardDeviations_[i] = weightedCovariance[i].diagonal().cwiseSqrt().asDiagonal();
    }

    ActionCovarianceTrajectory tmp = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionStepSize_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim)
    );
    for (size_t i = 0; i < predictionStepSize_ - 1; i++) {
        tmp[i] = - stepSize * (priorStandardDeviations_[i] - weightedStandardDeviations_[i]);
    }

    ActionCovarianceTrajectory updateCovariance_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionStepSize_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim)
    );
    for (size_t i = 0; i < predictionStepSize_ - 1; i++) {
        const Eigen::MatrixXd updatedStandardDeviation_ = 0.5 * (tmp[i] + (tmp[i].transpose() * tmp[i] + 4.0 * priorStandardDeviations_[i].transpose() * priorStandardDeviations_[i]).cwiseSqrt());
        updateCovariance_[i] = updatedStandardDeviation_ * updatedStandardDeviation_;
    }

    return std::make_pair(updateMean_, updateCovariance_);
}

ActionMeanTrajectory MPPI::interpolate(
    const ActionMeanTrajectory& actionMeanTrajectory1,
    const ActionMeanTrajectory& actionMeanTrajectory2,
    const double& ratio
) const
{
    ActionMeanTrajectory interpolatedActionMeanTrajectory_ = ActionMeanTrajectory::Zero(predictionStepSize_, actionMeanTrajectory1.cols());
    for (size_t i = 0; i < predictionStepSize_; i++) {
        interpolatedActionMeanTrajectory_.row(i) = (1.0 - ratio) * actionMeanTrajectory1.row(i) + ratio * actionMeanTrajectory2.row(i);
    }

    return interpolatedActionMeanTrajectory_;
}

ActionCovarianceTrajectory MPPI::interpolate(
    const ActionCovarianceTrajectory& actionCovarianceTrajectory1,
    const ActionCovarianceTrajectory& actionCovarianceTrajectory2,
    const double& ratio
) const
{
    int numCovarianceMatrices = predictionStepSize_ - 1; // 적절한 크기로 설정
    int covarianceDim = actionCovarianceTrajectory1[0].rows();

    ActionCovarianceTrajectory interpolatedActionCovarianceTrajectory_(numCovarianceMatrices, Eigen::MatrixXd::Zero(covarianceDim, covarianceDim));
    for (size_t i = 0; i < predictionStepSize_; i++) {
        interpolatedActionCovarianceTrajectory_[i] = (1.0 - ratio) * actionCovarianceTrajectory1[i] + ratio * actionCovarianceTrajectory2[i];
    }

    return interpolatedActionCovarianceTrajectory_;
}


// 테스트 함수
void MPPI::testComputeNormalPDFValue() {
    // 예시 데이터 설정
    ActionMeanTrajectory x(4, 2);
    x << 0.9, 0.0,
         1.1, 0.0,
         1.0, 0.0,
         0.7, 0.0;

    ActionMeanTrajectory mean(4, 2);
    mean << 4.8, 0.05,
            5.1, 0.07,
            4.9, 0.06,
            5.0, 0.08;

    ActionCovarianceTrajectory var(4);
    var[0] = (Eigen::MatrixXd(2, 2) << 0.2, 0.01, 0.01, 0.005).finished();
    var[1] = (Eigen::MatrixXd(2, 2) << 0.2, 0.01, 0.01, 0.005).finished();
    var[2] = (Eigen::MatrixXd(2, 2) << 0.2, 0.01, 0.01, 0.005).finished();
    var[3] = (Eigen::MatrixXd(2, 2) << 0.3, 0.02, 0.02, 0.007).finished();

    // // PDF 값 계산
    ActionMeanTrajectory pdf = computeNormalPDFValue(x, mean, var);

    // // 결과 출력
    std::cout << "PDF Values:\n" << pdf << std::endl;
}


} // namespace trajectory

} // namespace mppi
