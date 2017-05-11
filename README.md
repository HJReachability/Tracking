# Tracking
Fast and Safe Tracking (FaSTrack): effectively blending fast planning methods with slower, reacbability-based safety guarantees for online safe trajectory planning.

## Repository organization
Code in this repository is broken into two sections:
1. `code/` contains MATLAB code related to the original FaSTrack project
2. `ros/` contains a Robot Operating System (ROS) implementation of the FaSTrack idea that incorporates multiple planners hierarchically

## Usage
### MATLAB
_Insert instructions here._

### ROS
First, make sure you have ROS installed on your system. The project was developed in Jade, but it should be compatible with anything past Hydro. Please let us know if you have any compatibility issues.

Other dependencies:
* (Eigen)[https://eigen.tuxfamily.org] -- a header-only linear algebra library for C++
* Gtest -- Google's C++ unit testing library

To build, open a terminal window and navigate to the `ros/` directory. Then run:
```
catkin_make
```

Every time you open a new terminal, you'll have to tell ROS how to find this package. Do this by running the following command from the `ros/` directory:
```
source devel/setup.bash
```

To run a demo, type:
```
roslaunch [name-of-demo].launch
```
