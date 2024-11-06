# Stein Variational Guided MPPI

    svg_mppi_planner
    ├── include
    |   └── mppi_planner
    |      ├── common.hpp                         # data type definition for mppi
    |      ├── svg_mppi_planner_ros.hpp           # ROS dependencies
    |      └── svg_mppi.hpp                       # mppi implementation 
    └── src/
        ├── svg_mppi_planner_node.cpp             #
        ├── svg_mppi_planner_ros.cpp              #
        └── svg_mppi.cpp                          #

# Implementation

Varnila MPPI core sudo code

![](./icons/varnila_mppi_sudo.png){: .align-center width="500" height="300"}

1st loop: calculate_sample_costs

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
    - calculate_state_cost

2nd loop

```py
    for i from 0 to N - 1:
        # Update control sequence based on weighted samples
        u_i = u_i + H_inv * G * sum_over_k(exp(-S_tilde(τ_k) / λ) * ε_i,k) / sum_over_k(exp(-S_tilde(τ_k) / λ))
```

- calculate_sample_weights
