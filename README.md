# CarND-Controls-MPC
Model Predictive Controller for Udacity Self-Driving Car Simulation



## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`. See Run instructions for other options

## Basic Run Instructions
There are two options to run the controller. 
* Options 1: Default parameter `./mpc`
* Options 2: Parameter from command line `./mpc $N $dt $T $w_cte $w_epsi $w_v $w_delta $w_a $w_ddelta $w_da`
where
  * $N is the number of look-ahead steps
  * $dt is the sampling interval
  * $T is the latency.
  * $w_cte $w_epsi $w_v $w_delta $w_a $w_ddelta $w_da are optimization parameters to be explained in the Model section


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
    

## Model Implementation Details
Details of the model, the choice of parameters, and the latency handling method are described below:

### Model
The vehicle model implemented is a global kinematic model and the following state variables are used for the vehiclde:
$$x_{t+1} = x_t + v_t \cos(\psi_t) dt$$

$$y_{t+1} = y_t + v_t \sin(\psi_t) dt$$

$$\psi_{t+1} = \psi_t + \frac{v_t}{L_f}\delta_t dt$$

$$v_{t+1} = v_t + a_t dt$$

To calculate control policy, we also calculate two errors, the cross track error (CTE) and the orientation error:
$$\sf{cte}_{t} = y_t - f(x_t)$$
where $f(x_t)$ is the reference line,
$$\sf{e\psi}_t = \psi_t - \arctan(f'(x_t))$$ 

The model predictive controls is given by the solution that minimizes the following objective function 

$$\sum_{t=0}^{N-1} (w_{\sf{cte}} \sf{cte}_{t}^2 + w_{\sf{e\psi}} \sf{e{\psi}}_{t}^2 + w_{v} (v_{t}-v_{\sf{ref}})^2) + \sum_{t=0}^{N-2} (w_\delta \delta_t^2 + w_a a_t^2) + \sum_{t=0}^{N-3} (w_{d\delta}(\delta_{t+1}-\delta_t)^2 + w_{da}(a_{t+1}-a_t)^2)$$


### Parameters
Our choice of the MPC parameters are:
$N=10$, $d_t=0.2$, $w_{\sf{cte}} = 2$, $w_{\sf{e\psi}}=1$, $w_{v} = 10$, $w_\delta=1000$, $w_a=1000$, $w_\ddelta=1000$, $w_da=1000$.
Based on timing in the code, the average update time is around 0.2s, thus we choose $d_t=0.2$. This is also set as the latency. A large $N$ will leads to a better control action in each iteration but requires longer time to compute and a small $N$ will do the opposite. We choose $N=10$ as it seems to be a good balance between the two requirement based on past iterations with $N=5$, $N=10$ and $N=15$.


### Latency

To handle the latency, instead of using the current state values returned from the simulator, we set the initial $x_0$, $y_0$, $\psi_0$ and $v_0$ as the predicted value after $T$. 
Denote the state values returned from the simulator as $\bar{x}$, $\bar{y}$, $\bar{\psi}$, $\bar{v}$, $\bar{\delta}$ and $\bar{a}$. We set the initial values as:
$$x_{0} = \bar{x} + \bar{v} \cos(\bar{\psi}) T$$

$$y_{0} = \bar{y} + \bar{v} \sin(\bar{\psi}) T$$

$$\psi_{0} = \bar{\psi} + \frac{\bar{v}}{L_f}\bar{\delta} T$$

$$v_{0} = \bar{v} + \bar{a} T$$

Our choice of latency is $T=0.2$ based on timing information. 





