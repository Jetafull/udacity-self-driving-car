# CarND-Controls-MPC

Self-Driving Car Engineer Nanodegree Program

## Quick Start

1. Create a docker image with the dockerfile: `docker build -f yourdockeraccount/carnd-mpc .`
2. Run the image with port open to communicate with the simulator
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

## Polynomial Fitting and MPC Preprocessing

## Model Predictive Control with Latency