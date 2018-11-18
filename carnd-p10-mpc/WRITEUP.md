# CarND-Controls-MPC

Self-Driving Car Engineer Nanodegree Program

## Quick Start

Create a docker image with the dockerfile:

```bash
docker build -f yourdockeraccount/carnd-mpc .
```

Run the image with port open to communicate with the simulator

```bash
docker run -it -p 4567:4567 carnd-mpc bash
```

## Model Predictive Control

In this project I implemented the global kinematic model to automatically control the car driving.

The state vector includes

1. `x`: x-axis position of the car in the map
2. `y`: y-axis position of the car in the map
3. `psi`: orientation of the vehicle
4. `v`: velocity of vehicle
5. `cte`(cross track error): the error between the center of the road and the vehicle's position
6. `e_psi`: the orientation error

The actuators are

1. `delta`: steering angle
2. `a`: throttle

The cost function includes

1. Cumulative sum of cross track error and orientation error.
2. The magnitude of the actuators.
3. The temporal difference of the control inputs between the next state and current state.

The update steps are as follows:

1. The controller receives the current state and the reference trajectory in polynomial.
2. The solver find the optimal actuators based on the motion model in the next T seconds (N steps, `dt` interval), the cost function, and the constraints.
3. The controller send back the actuators as control input to the vehicle.

## Timestep Length and Elapsed Duration (N & dt)

The timestep length `N=15` and elapsed duration is `dt=0.05s`. Total time length is 0.75s. I tried the following combinations:

1. N=15 & dt=0.05
2. N=10 & dt=0.1
3. N=20 & dt=0.2

I chose the first combination for two reasons;

1. Smaller dt can gives more frequent control in same amount of time and gives finer control of the vehicle.
2. Smaller total time length is preferred to reduce the difficulty due to the environment change.

## Polynomial Fitting and MPC Preprocessing

I converted the waypoints to the vehicle coordinates. Then I fitted a order 3 polynomial and pass the coefficients into the solver.

One trick is the delta value sent from the simulator is to the opposite sign of the delta in the course. Therefore I modified the motion update model accordingly (see `state[2]` and `state[5]` in the code snippet below).

## Model Predictive Control with Latency

To account for the latency issue, I simply update the state using the latency interval and then pass the updated state into the controller solver:

```cpp
Eigen::VectorXd state(6);

const double x0 = 0;
const double y0 = 0;
const double psi0 = 0;
const double cte0 = coeffs[0] - y0;
const double epsi0 = psi0 - atan(coeffs[1]);

const double latency = 0.1;
const double Lf = 2.67;
state[0] = x0 + v * cos(psi0) * latency;
state[1] = y0 + v * sin(psi0) * latency;
state[2] = psi0 - v * delta / Lf * latency;
state[3] = v + a * latency;
state[4] = cte0 + v * sin(epsi0) * latency;
state[5] = epsi0 - v * delta / Lf * latency;

auto result = mpc.Solve(state, coeffs);
```