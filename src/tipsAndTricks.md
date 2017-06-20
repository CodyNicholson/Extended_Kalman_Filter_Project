# Tips & Tricks

### Summary of What Needs to Be Done

1. In tools.cpp, fill in the functions that calculate root mean squared error (RMSE) and the Jacobian matrix.
2. Fill in the code in FusionEKF.cpp. You'll need to initialize the Kalman Filter, prepare the Q and F matrices for the prediction step, and call the radar and lidar update functions.
3. In kalman_filter.cpp, fill out the Predict(), Update(), and UpdateEKF() functions.

### Tips and Tricks

Review the Previous Lessons

- Review the previous lessons! Andrei, Dominik and co. have given you everything you need. In fact, you've built most of an Extended Kalman Filter already! Take a look at the programming assignments and apply the techniques you used to this project.

#### No Need to Tune Parameters

- The R matrix values and Q noise values are provided for you. There is no need to tune these parameters for this project. In the unscented Kalman Filter lectures, we'll discuss how to determine these parameters.

##### Initializing the State Vector

- You'll need to initialize the state vector  with the first sensor measurement.
- Although radar gives velocity data in the form of the range rate ρ, a radar measurement does not contain enough information to determine the state variable velocities v_x and v_y. You can, however, use the radar measurements ρ and ϕ to initialize the state variable locations p_x and p_y.

#### Calculating y = z - H * x'

For lidar measurements, the error equation is **y = z - H * x'**. For radar measurements, the functions that map the x vector [px, py, vx, vy] to polar coordinates are non-linear. Instead of using H to calculate **y = z - H * x'**, for radar measurements you'll have to use the equations that map from cartesian to polar coordinates: **y = z - h(x')**.

#### Normalizing Angles

In C++, **atan2()** returns values between -pi and pi. When calculating phi in **y = z - h(x)** for radar measurements, the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. The Kalman filter is expecting small angle values between the range -pi and pi. HINT: when working in radians, you can add 2π or subtract 2π until the angle is within the desired range.

#### Avoid Divide by Zero throughout the Implementation

Before and while calculating the Jacobian matrix Hj, make sure your code avoids dividing by zero. For example, both the x and y values might be zero or **px * px + py * py** might be close to zero. What should be done in those cases?

#### Test Your Implementation

Test! We're giving you the ability to analyze your output data and calculate RMSE. As you make changes, keep testing your algorithm! If you are getting stuck, add print statements to pinpoint any issues. But please remove extra print statements before turning in the code.

### Ideas for Standing out!

The Kalman Filter general processing flow that you've learned in the preceding lessons gives you the basic knowledge needed to track an object. However, there are ways that you can make your algorithm more efficient!

- Dealing with the first frame, in particular, offers opportunities for improvement.
- Experiment and see how low your RMSE can go!
- Try removing radar or lidar data from the filter. Observe how your estimations change when running against a single sensor type! Do the results make sense given what you know about the nature of radar and lidar data?
- We give you starter code, but you are not required to use it! You may want to start from scratch if: You want a bigger challenge! You want to redesign the project architecture. There are many valid design patterns for approaching the Kalman Filter algorithm. Feel free to experiment and try your own! You want to use a different coding style, eg. functional programming. While C++ code naturally tends towards being object-oriented in nature, it's perfectly reasonable to attempt a functional approach. Give it a shot and maybe you can improve its efficiency!
