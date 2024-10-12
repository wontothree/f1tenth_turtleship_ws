// Anthony Garcia

#pragma once

#include <memory>  // std::unique_ptr를 위해 추가
#include <random>  // std::normal_distribution를 위해 추가
#include <grid_map_core/GridMap.hpp>
#include <omp.h>

#include "mppi_planner/mppi_types.hpp"

namespace mppi {

namespace cpu {

class MPPIBase
{
private:
    // local cost map object
    grid_map::GridMap localCostMap_;

    // global cost map object
    grid_map::GridMap globalCostMap_;

    // action trajectory mean
    ActionTrajectory actionTrajectoryMean_;

    // action trajectory covariance
    ActionTrajectoryCovariance actionTrajectoryCovariance_;

    // normal distribution pointor
    std::unique_ptr<std::vector<std::array<std::normal_distribution<>, ACTION_SPACE::dim>>> normalDistributionPtr_;

    // action trajectory with random noise
    std::vector<std::mt19937> rngs_;
    ActionTrajectoryBatch actionTrajectoryWithNoise_;

    int predictionHorizon_;
    int sampleNum_;

public:
    MPPIBase();

    /*
     * @brief set local cost map
     */
    void setLocalCostMap(const grid_map::GridMap& localCostMap);

    /*
     * @brief set global cost map
     */
    void setGlobalCostMap(const grid_map::GridMap& globalCostMap);

    /*
     * @brief set action trajectory mean
     */
    void setActionTrajectoryMean(const ActionTrajectory& actionTrajectoryMean);

    /*
    * @brief set action trajectory covariance
    */
    void setActionTrajectoryCovariance(const ActionTrajectoryCovariance& ActionTrajectoryCovariance);

    /**
     * @brief 주어진 평균 제어 시퀀스 및 공분산 행렬에 따라 무작위 샘플링을 수행합니다.
     *
     * 이 함수는 주어진 평균 제어 시퀀스와 공분산 행렬을 사용하여
     * 노이즈가 추가된 제어 시퀀스를 생성합니다. 생성된 샘플은 제어 입력
     * 제약 조건을 만족하도록 클리핑됩니다. 또한, 샘플링은 비편향
     * 샘플링과 편향 샘플링으로 구분되어 수행됩니다.
     *
     * @param control_seq_mean 평균 제어 시퀀스를 나타내는 ControlSeq 객체입니다.
     * @param control_seq_cov_matrices 각 제어 시퀀스에 대한 공분산 행렬을 포함하는 ControlSeqCovMatrices 객체입니다.
     *
     * @note 샘플링 과정은 OpenMP를 이용하여 병렬로 수행됩니다.
     */
    void sampleActionTrajectoryWithNoise(const ActionTrajectory& actionTrajectoryMean, const ActionTrajectoryCovariance actionTrajectoryCovariance);


    /**
    * @brief 일정한 제어 시퀀스 공분산 행렬을 생성합니다.
    *
    * 이 함수는 각 행렬의 대각선 요소가 입력 매개변수 `diagonal`에 지정된 값으로 설정된 제어 공분산 행렬 시퀀스를 생성합니다.
    * 비대각선 요소는 0으로 초기화됩니다.
    *
    * @param diag 대각선 요소의 값을 나타내는 double형 배열로,
    *             공분산 행렬의 크기는 제어 공간의 차원과 같습니다.
    * @return 지정된 대각선 값을 가진 공분산 행렬을 포함하는 ActionTrajectoryCovariance 객체를 반환합니다.
    */
    ActionTrajectoryCovariance getConstantActionTrajectoryCovariance(const std::array<double, ACTION_SPACE::dim>& diagonal) const
    {
        ActionTrajectoryCovariance actionTrajectoryCovariance_ = getZeroActionTrajectoryCovariance();
        for (auto& covariance: actionTrajectoryCovariance_)
        {
            for (size_t i = 0; i < ACTION_SPACE::dim; i++)
            {
                covariance(i, i) = diagonal[i];
            }
        }
        return actionTrajectoryCovariance_;
    }
    ActionTrajectoryCovariance getZeroActionTrajectoryCovariance() const
    {
        int predictionHorizon_ = 10; // 임시
        return ActionTrajectoryCovariance(predictionHorizon_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim));
    }
};

} // namespace cpu

} // namespace mppi
