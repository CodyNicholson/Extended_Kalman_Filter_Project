# Extended Kalman Filter Project

I built this Extended Kalman filter using C++ to estimate the state of a moving object of interest with noisy lidar and radar measurements

A standard Kalman filter can only handle linear equations. Both the Extended Kalman filter and the Unscented Kalman filter allow you to use non-linear equations; the difference between EKF and UKF is how they handle non-linear equations.

To make state estimation on nonlinear systems I had to refactor the original Kalman Filter into an Extended Kalman filter that can linearize the system under investigation around its current state and force the filter to use this linearized version of the system as a model. However, this filter may become unstable and results may be biased.

All Kalman filters have the same three steps:

1. Initialization
2. Prediction
3. Update
