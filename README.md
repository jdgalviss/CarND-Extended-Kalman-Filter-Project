# Extended Kalman Filter Implementation in c++

### In this project, sensor fusion is performed by implemented Kalman Filter and Extended Kalman Filter on Lidar and Radar measurements to estimate with a higher accuracy the position and velocity of a vehicle. Results can be seen on:
* [EKF Video](https://youtu.be/GWJFpWi_XQM)

---

The goals / steps of this project are the following:
* Read current sensor data from the [simulator](https://github.com/udacity/self-driving-car-sim/releases)
* Linearize model using first order Taylor expansion.
* Perform Kalman filter steps: Prediction and Measurement Update (correction) on data from radar and lidar.
* Calculate Mean Squared Error respect to ground truth data provided by the simulator.

[//]: # (Image References)

[image1]: ./output_imgs/ekf_1.png "ekf1"
[image2]: ./output_imgs/ekf_2.png "ekf2"
[image3]: ./output_imgs/ekf_lidar.png "ekf lidar"
[image4]: ./output_imgs/ekf_radar.png "ekf radar"



In this project kalman filter is used to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---


## Results
The results of the implementation of the kalman filter can be seen in this [Video](https://youtu.be/GWJFpWi_XQM). 

![alt text][image1]

However, it is of great interest to analyze the performance of each separate sensor, if we ignore data from radar, we would get following result:
![alt text][image3]

And if we were to ignore data from lidar, the result would be:
![alt text][image4]

As expected, the accuracy of the radar is infirior to the lidar, this was expected since, radar is escencially more noisy than lidar, it is however more robust in hard weather conditions. 

It can be clearly seen how fusioning these two sensors, we get an important error reduction, this is why Kalman Filters are so important, with them we can take two noisy sensors and boost their accuracy to get preciser and robuster state estimations.



## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

