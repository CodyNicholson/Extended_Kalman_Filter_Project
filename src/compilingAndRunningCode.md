# Compiling & Running Code

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

- mkdir build
- cd build
- cmake ..
- make
- ./ExtendedKF

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x ["estimate_y"] <= kalman filter estimated position y ["rmse_x"] ["rmse_y"] ["rmse_vx"] ["rmse_vy"]
