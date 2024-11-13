# Research Note

*08/12/2024*

정권 교체가 있었다.

```md
개인적으로 드리고 싶은 말이 있습니다.

대회는 앞으로 몇 달 안 남은 시점에 있습니다. 개인적으로는 군대 가기 전 마지막 마지막 프로젝트입니다.

이번 대회를 위한 팀장을 아직 정하지 않았는데요, 제가 팀장을 맡아도 괜찮을까요?

책임감과 추진력을 가지고 프로젝트에 임해보고 싶습니다.

팀장으로서 가장 중요한 역할은 팀원 모두가 각자의 역할을 잘할 수 있도록 돕는 일이라고 생각합니다. 

또한 제가 먼저 비전, 슬램, 제어 등 모든 분야를 깊게 이해하고 팀 모두가 공유할 수 있도록 하는 일이라고 생각합니다. 

이를 위해서 평소보다 적은 학점을 수강할 것이며 대부분의 시간을 여기에 집중시킬 계획입니다.
```

*09/01/2024*

한 명의 팀원이 이탈하여 세 명이서 프로젝트를 진행하게 되었다.

*09/26/2024*

Lidar가 도착했다.

*09/28/2024*

F1tenth 진행 사항이 없어서 교수님께 혼이 났다. 이제 cart pole project가 마무리되었으니 f1tenth에 시동을 걸어야겠다.

*2024.09.28*

#1 workspace

I made ros2 worksapce for f1tenth.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
colcon build
```

#2 Turtlesim

I will control turtle using keyborad direction in turtlesim.

```bash
sudo apt update
sudo apt install ros-foxy-turtlesim

ros2 pkg executables turtlesim

ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

- package : turtlesim
- node : turtlesim_node, turtle_teleop_key

turtle_teleop_key

- input : user keyboard direction input
- output : '/turtle1/cmd_vel'

You can see rqt graph

```bash
rqt_graph
```

#3 Joystick

You can find connected device to your computer using the following command

```bash
lsusb

ls /dev/input*
```

Useful app for joystick - jstest-gtk

```bash
sudo apt install joystick jstest-gtk ros-kinetic-joy
sudo apt purge ros-kinetic-teleop-twist-joy
cd ~/catkin_ws/src
git clone https://github.com/robotpilot/teleop_twist_joy.git
cd ~/catkin_ws && catkin_make
sudo apt install jstest-gtk
jstest-gtk
```

This repository provides transform from '/joy' to 'cmd_vel'. The package comes with the teleop_node that republishes sensor_msgs/msg/Joy messages as scaled geometry_msgs/msg/Twist messages.

[teleop_twist_joy foxy](https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/)

```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

Insert the usb of joystic to my computer. You can check if input of joystic is received.

```bash
ros2 topic echo \joy

ros2 topic echo cmd_vel
```

Though input of joystic control angular z, it can not change the other values. I think button map is not correct.

```bash
---
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.0
---
```

Unfortunately, this repository doesn't provide interface for my joystick - F7 10 logitect

https://github.com/husarion/joy2twist/blob/ros2/joy2twist/package.xml

*2024.09.29*

Let's start from turtlesim. What is prinple that I can control turtle using 'turtle_teleop_key' node in turtlesim?

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

```bash
ros2 node list
// ...
/teleop_turtle
/turtlesim
```

```bash
ros2 topic list
// ...
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

```bash
ros2 topic echo /turtle1/cmd_vel
// ...
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

When I press upper direction in keyboard, x = 2.0 in linear.

When I press down direction in keyboard, x = -2.0 in linear.

When I press right direction in keyboard, z = -2.0 in angular.

When I press left direction in keyboard, z = 2.0 in angular.

In order to control turtle in turtlesim by joystick, I should transform input of joystick to /turtle1/cmd/vel (geometry_msgs/msg/Twist)

I conclude that it is more fast to myself make package for f7 10 jostick.

1. Input of joystick F7 10 to '/joy' (sensor_msgs/Joy)  
2. '/joy' (sensor_msgs/Joy) to '/cmd_vel' (geometry_msgs/Twist)

Process 1 is possible for ROS2 standard package. 

```bash
ros2 run joy joy_node
```

I shoud develop package for process 2 - joystick_f710_controller

I get many help from [GITHUB - joy2twist](https://github.com/husarion/joy2twist/tree/ros2/joy2twist/include/joy2twist)

https://github.com/f1tenth/f1tenth_system

*2024.10.01*

What do I need to do to control vesc driver

#4 Vesc

```bash
/commands/motor/brake # *

/commands/motor/current
/commands/motor/duty_cycle 
/commands/motor/position # 우리 motor가 hall sensor less라 측정할 수 없다.
/commands/motor/speed # v

/commands/servo/position # *

/sensors/core
/sensors/imu
/sensors/imu/raw
/sensors/servo_position_command
```

*2024.10.02*

The object that i shoud do

```bash
/commands/motor/brake
/commands/servo/position
```

I wonder What is difference between `/commands/motor/duty_cycle` and `/commands/motor/speed`. Is it sufficient to give one among the supposing fuor topics? I will use speed control at first.

```bash
/commands/motor/current
/commands/motor/duty_cycle
/commands/motor/position
/commands/motor/speed
```

just memo. Why do it work only when i run this node? To understand this mechanism, i need to know how to work [this node](https://github.com/f1tenth/vesc/tree/1952e799110f5c4eed82c68a6172bfcafd9998ac).

```bash
ros2 launch vesc_driver vesc_driver_node.launch.py
```

solution

*10/06/2024*

Cart Pole 프로젝트가 끝나서 여유가 생기기 무섭게 교수님께 공식 메일이 왔다. F1tenth 팀이 개발 속도가 너무 느리다는 말씀과 함께 tension injection을 해주셨다.

개발을 시작한지 일주일째이다. 일주일 동안 돌아가는 시스템을 구축했다.

- Joystick으로 turtleshim에서 turtle 제어하기
- Joystick으로 차량 제어하기
- 간단한 pid planner 넣어서 자율주행하기

다음주에는 SVGD MPPI를 구현할 예정이다. 구현이 된다면 ICCAS에 등록할 것이다. 다음주까지 목표

- local_costmap_generator
- svgd mppi planner
- slam toolbox

local costmap을 만들기 위해서는 ego_racecar/base_link를 publish할 수 있어야 한다. 아직 이걸 어떻게 사용한다는지에 대한 명확한 감이 없긴하다. IMU 센서로부터 받을 수 있을 것 같다.

TF는 IMU로부터 생성하는 건 아닌 것 같다. 다음 post들을 숙지하자.

- [Introducing tf2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [Writing a static broadcaster (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html)
- [Writing a broadcaster (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html)
- [Writing a listener (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)

https://with-rl.tistory.com/entry/ROS2-Transformation-System-TF2

*10/07/2024*

In your real development process you shouldn’t have to write this code yourself and should use the dedicated tf2_ros tool to do so. tf2_ros provides an executable named static_transform_publisher that can be used either as a commandline tool or a node that you can add to your launchfiles.

Publish a static coordinate transform to tf2 using an x/y/z offset in meters and roll/pitch/yaw in radians. In our case, roll/pitch/yaw refers to rotation about the x/y/z-axis, respectively.

```bash
ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
```

예를 들면, 차량에서 lidar의 물리적인 위치와 차량의 중심의 위치가 일치할 때,

```bash
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 "ego_racecar/laser" "ego_racecar/base_link"
```

"ego_racecar/base_link"는 차량의 질량 중심이 아닌 기하학적 중심을 의미한다. (질량 중심과 기하학적 중심이 일치하도록 설계하는 경우가 많다.)

만약 라이다의 위치가 차량의 중심보다 9cm 앞에 있다면 다음과 같이 명령어를 구성해야 한다.

```bash
ros2 run tf2_ros static_transform_publisher 0.09 0.0 0.0 0.0 0.0 0.0 "ego_racecar/laser" "ego_racecar/base_link"
```

이로써 tf_static/ 토픽은 발행할 수 있게 되었다. 이제 tf/를 publish할 수 있는 방법을 찾아야 한다.

- /tf 토픽: 동적인 좌표 변환 정보가 퍼블리시되는 토픽. 예를 들어, 움직이는 로봇의 좌표 변환이 이 토픽을 통해 주기적으로 업데이트됩니다. 거북이의 위치와 회전 정보가 이 토픽에 퍼블리시됩니다.
- /tf_static 토픽: 고정된 좌표 변환 정보가 퍼블리시되는 토픽입니다. 움직이지 않는 정적인 좌표 변환은 이 토픽에 한 번만 퍼블리시됩니다.

local costmap generator를 구현하려는데 tf가 자꾸 발목을 잡는다.

```bash
ros2 run tf2_ros tf2_monitor
```

local costmap...

lidar point가 내가 정의한 costmap 안에 있는지 없는지 여부가 가장 중요하다.

- lidar point가 costmap 내부에 있다 -> occupied
- lidar point가 costmap 외부에 있다 -> empty

occupied된 점에 해당하는 index를 저장한다.

```cpp
std::vector<grid_map::Index> LocalCostmapGenerator::pclToCostmap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl,
    grid_map::GridMap* costmap
) const
{ 
    // 
    return occupiedIndices;
}
```

이 함수는 costmap

tf topic

```bash
---
transforms:
- header:
    stamp:
      sec: 1728292305
      nanosec: 845752009
    frame_id: map
  child_frame_id: ego_racecar/base_link
  transform:
    translation:
      x: 21.603648091272074
      y: 18.44798193306152
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.8273751304988164
      w: 0.5616497070524176
---
transforms:
- header:
    stamp:
      sec: 1728292305
      nanosec: 845752009
    frame_id: ego_racecar/base_link
  child_frame_id: ego_racecar/laser
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1728292305
      nanosec: 845752009
    frame_id: ego_racecar/front_left_hinge
  child_frame_id: ego_racecar/front_left_wheel
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: -0.007836380861107542
```

tf_static : rotation이 시간에 따라 변하지 않는 것을 확인할 수 있다.

```bash
- header:
    stamp:
      sec: 1728292426
      nanosec: 844756926
    frame_id: ego_racecar/base_link
  child_frame_id: ego_racecar/back_right_wheel
  transform:
    translation:
      x: 0.0
      y: -0.12065
      z: 0.0508
    rotatio
```

*10/09/2024*

라이다를 배터리와 연결된 승압기에 연결하였는데도 라이다에 전원이 들어오지 않았다. 승압기에 외부 전원을 주니 라이다에 전원이 들어왔다. 배터리에 문제가 있는 것이었다. 배터리를 충전기에 연결해보았다.

배터리가 죽은 것을 확인할 수 있었다.

*10/23/2024*

MPPI solver sudo code

```cpp
std::pair<StateTrajectory, ActionTrajectory> MPPI::solve(const State& currentState)
{
    // number of collision
    int collisionNum_ = 0;

    // 이전 시퀀스를 각 단계의 평균으로 설정한다. for warm start
    ControlTrajectory controlTrajectoryMean_ = previousControlTrajectory_;
    ControlTrajectoryCovariance controlTrajectoryCovariance_ = previousControlTrajectoryCovariance_;
    ControlTrajectory rejectedControlTrajectoryMean_ = previousRejectedControlMean_;
    ControlTrajectoryCovariance rejectedControlTrajectoryCovariance_ = previousRejectedControlCovariance_;

    for (int n = 0; n < N; n++) // N : number of iteration
    {
        noisedControlTrajectorySamples_ = randomSampling(controlTrajectoryMean_, controlTrajectoryCovariance_);

        // 각 샘플의 거부 확률을 구한다. 새로 생성된 샘플들이 이전에 거부된 샘플들과 유사할수록 높은 거부 확률을 가진다.
        // sample rejection probabilities
        ControlTrajectory normalProbability = normalPdf(rejectedControlTrajectoryMean_, controlTrajectoryMean_, controlTrajectoryCovariance_);
        // double 타입의 벡터를 샘플 수만큼 생성하고, 0으로 초기화한다.
        std::vector<double> probabilities(sampleNum_, 0.0);
        for (size_t j = 0; j < sampleNum; j++) // 모든 샘플을 순회한다.
        {
            const ControlTrajectory probability = normalPdf(noisedControlTrajectorySamples_, rejectedControlTrajectoryMean_, rejectedControlTrajectoryCovariance_);
            // 이 확률값이 클수록 정규분포에서 많이 벗어난 것이며, 거부될 확률이 높다.
            probabilities[j] = (normalProbability + probability).cwiseInverse().prod();
        }

        // normalization probilities (summation = 1)
        const double probabilitySum_ = std::min(std::accumulate(probabilities.begin(), probabilities.end(), 0.0), 1e-10);
        for (size_t j = 0; j < sampleNum; j++) // 모든 샘플을 순회한다.
        {
            probabilities[j] /= probabilitySum_;
        }

        // 샘플 선택 및 업데이트
        const std::vector<int> selectedIndices_ = random_sample_choice(sampleNum, probabilities);
        shrink_copy_from(*inflated_samples_ptr_, selectedIndices_);


        // 비용 예측 및 계산
        // 현재 상태에 대한 샘플의 비용을 계산한다.
        // ------------------------------ 핵심 ----------------------------------
        auto [_cost, collision_costs] = calc_sample_costs(*prior_samples_ptr_, currentState);
        prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);
        collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });

        // 가중치 계산
        const auto [weights_eval, weights_reject] = calc_weights(*prior_samples_ptr_);


        // 가중 평균 및 표준 편차 계산
        const auto [mean_eval, sigma_eval] = weighted_mean_and_sigma(*prior_samples_ptr_, weights_eval);
        const auto [mean_reject, sigma_reject] = weighted_mean_and_sigma(*prior_samples_ptr_, weights_reject);

        // MD update
        const auto [new_mean, new_covs] = md_update(control_seq_mean, control_seq_cov_matrices, mean_eval, sigma_eval, step_size_);
        control_seq_mean = new_mean;
        control_seq_cov_matrices = new_covs;

        const auto [new_rejected_mean, new_rejected_covs] = md_update(rejected_mean, rejected_covs, mean_reject, sigma_reject, step_size_);
        rejected_mean = new_rejected_mean;
        rejected_covs = new_rejected_covs;
    }

    
    // Warm start for next control iteration
    // control trajectory 보간 함수 : warm_start_ratio를 고려하고 두 입력 사이의 값을 취한다.
    previousControlTrajectory_ = interpolate(previousControlTrajectory_, controlTrajectoryMean_, warm_start_ratio_);
    previousControlTrajectoryCovariance_ = interpolate(previousControlTrajectoryCovariance_, controlTrajectoryCovariance_, warm_start_ratio_);
    previousRejectedControlMean_ = interpolate(previousRejectedControlMean_, rejectedControlTrajectoryMean_, warm_start_ratio_);
    previousRejectedControlCovariance_ = interpolate(previousRejectedControlCovariance, rejectedControlTrajectoryCovariance_, warm_start_ratio_);

    const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());

    return std::make_pair(control_seq_mean, collision_rate);
}
```

*10/29/2024*

solver - random sampling

```cpp
// set normal distributions parameters
for (size_t i = 0; i < predictionHorizon_ - 1; i++) {
    for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
        // standard deviation in each time step
        const double standardDeviation_ = std::sqrt(actionTrajectoryCovariance_[i](j, j));

        // normal distribution for noise
        std::normal_distribution<>::param_type noiseNormalDistribution_(0.0, standardDeviation_);

        //
        (*normalDistributionPointer_)[i][j].param(noiseNormalDistribution_);
    }
}

// 
for (size_t i = 0; i < sampleNum_; i++) {
    // generate noise sequence
    for (size_t j = 0; j < predictionHorizon_; j++) {
        for (size_t k = 0; k < ACTION_SPACE::dim; k++) {
            // 수정해야 함
            actionNoiseSamples_[i](j, k) = (*normalDistributionPointer_)[j][k](randomNumberGenerators_[j]);
        }
    }

    // sampling action sequences with non-biased (around zero) sampling rate
    if (i < static_cast<size_t>((1 - nonBiasedSamplingRate_) * sampleNum_)) {
        // biased sampling (around actionTrajectoryMean_)
        noisedActionTrajectorySamples_[i] = actionTrajectoryMean_ + actionNoiseSamples_[i];
    } else {
        // non-biased sampling (around zero)
        noisedActionTrajectorySamples_[i] = actionNoiseSamples_[i];
    }

    // clip input with control input constraints
    for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
        for (size_t k = 0; k < predictionHorizon_ - 1; k++) {
            noisedActionTrajectorySamples_[i](k, j) = std::clamp(noisedActionTrajectorySamples_[i](k, j), minAction_[j], maxAction_[j]);
        }
    }
}
```

reference - normal_pdf

```cpp
ControlSeq ReverseMPPI::normal_pdf(const ControlSeq& x, const ControlSeq& mean, const ControlSeqCovMatrices& var) const {
    ControlSeq pdf = x * 0.0;
    for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
        pdf.row(i) = (x.row(i) - mean.row(i))
                          .cwiseProduct((var[i].diagonal().cwiseSqrt().asDiagonal().inverse() * (x.row(i) - mean.row(i)).transpose()).transpose())
                          .array()
                          .exp() /
                      std::sqrt(std::pow(2.0 * M_PI, CONTROL_SPACE::dim) * var[i].determinant());
    }
    return pdf;
}
```

computePDFValue

```cpp
ActionTrajectory ReverseMPPI::computePDFValue(
    const ActionTrajectory& observedValue,
    const ActionTrajectory& mean,
    const ActionTrajectoryCovariance& variance
) const {
    ActionTrajectory probabilityDensityFunctionValue = ActionTrajectory::Zero(observedValue.rows(), observedValue.cols());

    for (size_t i = 0; i < predictionStepSize_ - 1; i++) {
        // 차이 벡터 계산
        auto singleDeviation_ = observedValue.row(i) - mean.row(i);

        // 분산의 역행렬의 제곱근 계산
        auto inverseVariance_ = variance[i].diagonal().cwiseSqrt().asDiagonal().inverse();

        // 확률 밀도 함수의 지수 계산
        auto exponent = (inverseVariance_ * singleDeviation_.transpose()).array().exp();

        // 확률 밀도 함수 값 계산
        probabilityDensityFunctionValue.row(i) = singleDeviation_.cwiseProduct(exponent) /
            std::sqrt(std::pow(2.0 * M_PI, ACTION_SPACE::dim) * variance[i].determinant());
    }

    return probabilityDensityFunctionValue;
}
```

reference - random_sample_choice : 주어진 확률에 따라 임의의 샘플 인덱스를 선택하는 함수

```cpp
std::vector<int> PriorSamplesWithCosts::random_sample_choice(
  const size_t& num_samples,
  const std::vector<double>& probabilities
) {
    if (num_samples >= num_samples_) {
        std::cout << "Error: num_samples is larger than num_samples_." << std::endl;
        exit(1);
    }

    if (probabilities.size() != num_samples_) {
        std::cout << "Error: probability size is not equal to num_samples_." << std::endl;
        exit(1);
    }

    // choose random indices with probability
    discrete_dist_.param(std::discrete_distribution<>::param_type(probabilities.begin(), probabilities.end()));
    std::vector<int> indices(num_samples);
#pragma omp parallel for num_threads(thread_num_)
    for (size_t i = 0; i < num_samples; i++) {
        indices[i] = discrete_dist_(rngs_[omp_get_thread_num()]);
    }

    return indices;
}
```

sampleIndicesWithPDF

```cpp
std::vector<int> PriorSamplesWithCosts::sampleIndicesWithPDF(const size_t& num_samples, const std::vector<double>& probabilities) {
    if (num_samples >= num_samples_) {
        std::cout << "Error: num_samples is larger than num_samples_." << std::endl;
        exit(1);
    }

    if (probabilities.size() != num_samples_) {
        std::cout << "Error: probability size is not equal to num_samples_." << std::endl;
        exit(1);
    }

    // choose random indices with probability
    discrete_dist_.param(std::discrete_distribution<>::param_type(probabilities.begin(), probabilities.end()));
    std::vector<int> indices(num_samples);
    for (size_t i = 0; i < num_samples; i++) {
        indices[i] = discrete_dist_(randomNumberGenerators_[i]);
    }

    return indices;
}
```

reference - shrink_copy_from

```cpp
// copy part of samples that are selected from source samples
void PriorSamplesWithCosts::shrink_copy_from(const PriorSamplesWithCosts& source_samples, const std::vector<int>& indices) {
    // check: source samples size is larger than indices size because shrinked samples are stored in this class
    if (indices.size() != num_samples_) {
        std::cout << "Error: indices size is not equal to num_samples_." << std::endl;
        exit(1);
    }

    if (source_samples.prediction_horizon_ != prediction_horizon_) {
        std::cout << "Error: source_samples.prediction_horizon_ is not equal to prediction_horizon_." << std::endl;
        exit(1);
    }

    if (source_samples.num_samples_ < indices.size()) {
        std::cout << "Error: source_samples.num_samples_ is smaller than indices.size()." << std::endl;
        exit(1);
    }

#pragma omp parallel for num_threads(thread_num_)
    for (size_t i = 0; i < indices.size(); i++) {
        noised_control_seq_samples_[i] = source_samples.noised_control_seq_samples_[indices[i]];
        noise_seq_samples_[i] = source_samples.noise_seq_samples_[indices[i]];
        costs_[i] = source_samples.costs_[indices[i]];
    }
    set_control_seq_mean(source_samples.control_seq_mean_);
    set_control_seq_cov_matrices(source_samples.control_seq_cov_matrices_);
}
```

sampling

```cpp

````

reference - calc_sample_costs

사용되고 있는 사용자 정의 함수

- get_num_samples
- predict_state_seq
- state_cost

```cpp
std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(
  const PriorSamplesWithCosts& sampler,
  const State& local_init_state,
  const grid_map::GridMap& obstacle_map,
  StateSeqBatch* local_state_seq_candidates
) const {
    std::vector<double> costs(sampler.get_num_samples());
    std::vector<double> collision_costs(sampler.get_num_samples());

// Rollout for each control sequence
#pragma omp parallel for num_threads(thread_num_)
    for (size_t i = 0; i < sampler.get_num_samples(); i++) {
        // Predict state sequence
        local_state_seq_candidates->at(i) = predict_state_seq(sampler.noised_control_seq_samples_[i], local_init_state, obstacle_map);

        // calculate cost
        const auto [cost, collision_cost] = state_cost(local_state_seq_candidates->at(i), obstacle_map);
        costs.at(i) = cost;
        collision_costs.at(i) = collision_cost;
    }

    return std::make_pair(costs, collision_costs);
}
```

*10/31/2024*

```cpp
void MPPI::sampleNoisyActionTrajectory(
    const ActionMeanTrajectory& actionMeanTrajectory_,
    const ActionCovarianceTrajectory& actionCovarianceTrajectory_
)
{
    // set normal distributions parameters
    for (size_t i = 0; i < predictionHorizon_ - 1; i++) {
        for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
            // standard deviation in each time step
            const double standardDeviation_ = std::sqrt(actionTrajectoryCovariance_[i](j, j));

            // normal distribution for noise
            std::normal_distribution<>::param_type noiseNormalDistribution_(0.0, standardDeviation_);

            //
            (*normalDistributionPointer_)[i][j].param(noiseNormalDistribution_);
        }
    }

    // 
    for (size_t i = 0; i < sampleNum_; i++) {
        // generate noise sequence
        for (size_t j = 0; j < predictionHorizon_; j++) {
            for (size_t k = 0; k < ACTION_SPACE::dim; k++) {
                // 수정해야 함
                actionNoiseSamples_[i](j, k) = (*normalDistributionPointer_)[j][k](randomNumberGenerators_[j]);
            }
        }

        // sampling action sequences with non-biased (around zero) sampling rate
        if (i < static_cast<size_t>((1 - nonBiasedSamplingRate_) * sampleNum_)) {
            // biased sampling (around actionTrajectoryMean_)
            noisedActionTrajectorySamples_[i] = actionMeanTrajectory_ + actionNoiseSamples_[i];
        } else {
            // non-biased sampling (around zero)
            noisedActionTrajectorySamples_[i] = actionNoiseSamples_[i];
        }

        // clip input with control input constraints
        for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
            for (size_t k = 0; k < predictionHorizon_ - 1; k++) {
                noisedActionTrajectorySamples_[i](k, j) = std::clamp(noisedActionTrajectorySamples_[i](k, j), minAction_[j], maxAction_[j]);
            }
        }
    }
}
```

위 함수에서 멤버 변수를 반환하도록 할 수 있을까?

```cpp
const ActionTrajectory& MPPI::sampleNoisyActionTrajectory(
    const ActionMeanTrajectory& actionMeanTrajectory_,
    const ActionCovarianceTrajectory& actionCovarianceTrajectory_
)
{
    // set normal distributions parameters
    for (size_t i = 0; i < predictionHorizon_ - 1; i++) {
        for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
            // standard deviation in each time step
            const double standardDeviation_ = std::sqrt(actionTrajectoryCovariance_[i](j, j));

            // normal distribution for noise
            std::normal_distribution<>::param_type noiseNormalDistribution_(0.0, standardDeviation_);

            //
            (*normalDistributionPointer_)[i][j].param(noiseNormalDistribution_);
        }
    }

    // 
    for (size_t i = 0; i < sampleNum_; i++) {
        // generate noise sequence
        for (size_t j = 0; j < predictionHorizon_; j++) {
            for (size_t k = 0; k < ACTION_SPACE::dim; k++) {
                // 수정해야 함
                actionNoiseSamples_[i](j, k) = (*normalDistributionPointer_)[j][k](randomNumberGenerators_[j]);
            }
        }

        // sampling action sequences with non-biased (around zero) sampling rate
        if (i < static_cast<size_t>((1 - nonBiasedSamplingRate_) * sampleNum_)) {
            // biased sampling (around actionTrajectoryMean_)
            noisedActionTrajectorySamples_[i] = actionMeanTrajectory_ + actionNoiseSamples_[i];
        } else {
            // non-biased sampling (around zero)
            noisedActionTrajectorySamples_[i] = actionNoiseSamples_[i];
        }

        // clip input with control input constraints
        for (size_t j = 0; j < ACTION_SPACE::dim; j++) {
            for (size_t k = 0; k < predictionHorizon_ - 1; k++) {
                noisedActionTrajectorySamples_[i](k, j) = std::clamp(noisedActionTrajectorySamples_[i](k, j), minAction_[j], maxAction_[j]);
            }
        }
    }

    return noisedActionTrajectorySamples_;
}
```

*11/04/2024*

solve에서 사용되는 함수

|Index|Function Name||Description|
|---|---|---|---|
|1|sampleNoisedActionTrajectory|||
|2|computeNormalPDFValue|||
|3|sampleWithProbability|||
|4|shrink_copy_from|||
|5|computeSampleCost|||
|6||predictStateTrajectory||
|7||predictConstantSpeed||
|8||stateCost ->computeStateCost||
|9|computeWeights|||
|10|computeCostsWithControlTerm|||
|11|weightedMeanAndSigma->computeWeightedMeanAndCovariance|||
|12|updateMeanAndCovariance|||
|13|interpolate|||
|14|interpolate|||

*11/06/2024*

Varnila MPPI Sudo Code

```py
Given:
    K: Number of samples
    N: Number of timesteps
    (u0, u1, ..., uN-1): Initial control sequence
    Δt, x_t0, f, G, B, B_E: System/sampling dynamics
    φ, q, R, λ: Cost parameters
    u_init: Value to initialize new controls to

while task is not completed:
    for k from 0 to K - 1:
        x = x_t0
        for i from 1 to N - 1:
            # Update state based on dynamics with noise
            x_i+1 = x_i + (f + G * u_i) * Δt + B_E * ε_i,k * sqrt(Δt)
            
            # Accumulate the cost for trajectory
            S_tilde(τ_k) += φ(x_i, u_i, ε_i,k, t_i)

    for i from 0 to N - 1:
        # Update control sequence based on weighted samples
        u_i = u_i + H_inv * G * sum_over_k(exp(-S_tilde(τ_k) / λ) * ε_i,k) / sum_over_k(exp(-S_tilde(τ_k) / λ))

    # Apply the first control to actuators
    send u_0 to actuators
    
    # Shift control sequence
    for i from 0 to N - 2:
        u_i = u_i+1
    u_N-1 = u_init

    # Update the current state based on feedback
    Update current state
    
    # Check if task is completed
    check for task completion
```

Implementation

```py
    for k from 0 to K - 1:
        x = x_t0
        for i from 1 to N - 1:
            # Update state based on dynamics with noise
            x_i+1 = x_i + (f + G * u_i) * Δt + B_E * ε_i,k * sqrt(Δt)
            
            # Accumulate the cost for trajectory
            S_tilde(τ_k) += φ(x_i, u_i, ε_i,k, t_i)

```

- calculate_sample_costs
    - predict_state_trajectory
        - predict_constant_speed
        - predict_linear_speed
        - predict_reference_speed
    - calculate_state_costs

```py
    for i from 0 to N - 1:
        # Update control sequence based on weighted samples
        u_i = u_i + H_inv * G * sum_over_k(exp(-S_tilde(τ_k) / λ) * ε_i,k) / sum_over_k(exp(-S_tilde(τ_k) / λ))
```

- calculate_sample_weights

*11/13/2024*

random_sampling 함수 test
