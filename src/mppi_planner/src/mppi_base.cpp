// Anthony Garcia

#include <iostream>

#include "mppi_planner/mppi_base.hpp"

namespace mppi {

namespace cpu {

MPPIBase::MPPIBase()
{
    predictionHorizon_ = 10;
    sampleNum_ = 10;

    // action trajectory mean
    actionTrajectoryMean_ = Eigen::MatrixXd::Zero(predictionHorizon_ - 1, ACTION_SPACE::dim);

    // action trajectory covariance
    actionTrajectoryCovariance_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
        predictionHorizon_ - 1, Eigen::MatrixXd::Zero(ACTION_SPACE::dim, ACTION_SPACE::dim));
    
    // action trajectory with random noise
    actionTrajectoryWithNoise_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            sampleNum_, Eigen::MatrixXd::Zero(predictionHorizon_ - 1, ACTION_SPACE::dim));
}

/*
 * @brief set local cost map
 */
void MPPIBase::setLocalCostMap(const grid_map::GridMap& localCostMap) { localCostMap_ = localCostMap; }

/*
 * @brief set global cost map
 */
void MPPIBase::setGlobalCostMap(const grid_map::GridMap& globalCostMap) { globalCostMap_ = globalCostMap; }

/*
 * @brief set action trajectory mean
 */
void MPPIBase::setActionTrajectoryMean(const ActionTrajectory& actionTrajectoryMean)
{
    actionTrajectoryMean_ = actionTrajectoryMean;
}

/*
 * @brief set action trajectory covariance
 */
void MPPIBase::setActionTrajectoryCovariance(const ActionTrajectoryCovariance& actionTrajectoryCovariance)
{
    actionTrajectoryCovariance_ = actionTrajectoryCovariance;
}

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
void MPPIBase::sampleActionTrajectoryWithNoise(const ActionTrajectory& actionTrajectoryMean, const ActionTrajectoryCovariance actionTrajectoryCovariance)
{
    // set action trajectory mean
    setActionTrajectoryMean(actionTrajectoryMean);

    // set action trajectory covariance
    setActionTrajectoryCovariance(actionTrajectoryCovariance);

    // set normal distributions parameters
    for (size_t i = 0; i < predictionHorizon_ - 1; i++)
    {
        for (size_t j = 0; j < ACTION_SPACE::dim; j++)
        {
            // standard deviation in normaal distribution
            const double standardDeviation_ = std::sqrt(actionTrajectoryCovariance_[i](j, j));

            std::normal_distribution<>::param_type param(0.0, standardDeviation_);
            (*normalDistributionPtr_)[i][j].param(param);
        }
    }

    #pragma omp parallel for num_threads(4)
    for (size_t i = 0; i < sampleNum_; i++)
    {
        // generate action trajectory with noise
        for (size_t j = 0; j < predictionHorizon_ - 1; j++)
        {
            for (size_t k = 0; k < ACTION_SPACE::dim; k++)
            {
                actionTrajectoryWithNoise_[i](j, k) = (*normalDistributionPtr_)[j][k](rngs_[omp_get_thread_num()]);
            }
        }

        // sampling action trajectory with non-biased (around zero) sampling rate

        // clip input with action input constraints
    }
}

} // namespace cpu

} // namespace mppi
