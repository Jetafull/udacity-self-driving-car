# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Discussions

### Effects of PID Components

* Proportional (P): the control output is proportional to the cross track error. The corrective value is to the opposite direction to the CTE (cross track error). And its absolute value will be larger if the cross track error is larger.

* Integral (I): the control output is proportional to the temporal cumulative errors. If over time the cumulative error is larger, it will add more corrective value to the error from other parts.

* Derivative (D): the control output is based on the rate of error change. It can prevent the overshoot by partially cancelling out the corrective value from P.

### Final Parameters Tuning

I manually tuned the parameters as follows:

1. I tuned the coefficient for `P` first. Using the final P parameter the car can run in steady oscillation in straight lines.
2. I then tune the coefficients for `D` so that the oscillations have been significantly reduced. After tunning, the car can finish a complete run.
3. Finally I tune the `I` coefficient. The oscillations are further reduced.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
