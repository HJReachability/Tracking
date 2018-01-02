# Tracking
Fast and Safe Tracking (FaSTrack): effectively blending fast planning methods with slower, reacbability-based safety guarantees for online safe trajectory planning.

## Repository organization
Code in this repository is broken into two sections:
1. `code/` contains MATLAB code related to the original FaSTrack project
2. `ros/` contains a Robot Operating System (ROS) implementation of the FaSTrack idea that incorporates multiple planners hierarchically

Please note that the C++ / ROS implementation is deprecated here, and active development is continuing under the name [meta-fastrack](https://github.com/HJReachability/meta_fastrack)

## Usage
### MATLAB
_Insert instructions here._

### C++ / ROS
First, make sure you have ROS installed on your system. The project was developed in Jade, but it should be compatible with anything past Hydro. Please let us know if you have any compatibility issues.

`Tracking` currently depends upon the [crazyflie_clean](https://github.com/dfridovi/crazyflie_clean) repository, which contains drivers and utilities for the HSL's Crazyflie 2.0 testbed. However, since this part of the repository is deprecated it only works with release [v0.1](https://github.com/HJReachability/crazyflie_clean/releases).

Other dependencies:
* [Gtest](https://github.com/google/googletest) -- Google's C++ unit testing library
* [Eigen](https://eigen.tuxfamily.org) -- a header-only linear algebra library for C++
* [OMPL](http://ompl.kavrakilab.org) -- an open C++ library for motion planning (recommend v1.2.1 to avoid g++5 dependency)
* [MATIO](https://github.com/tbeu/matio) -- an open C library for MATLAB MAT file I/O
* [FLANN](http://www.cs.ubc.ca/research/flann/) -- an open source library for fast (approximate) nearest neighbors

You must begin by building and sourcing the `crazyflie_clean` repository. Instructions may be found in that project's README. To build `Tracking`, open a terminal window and navigate to the `ros/` directory. Then run:
```
catkin_make
```

Every time you open a new terminal, you'll have to tell ROS how to find this package. Do this by running the following command from the `ros/` directory:
```
source devel/setup.bash
```

`Tracking` includes two demos, one software and one hardware. To run the hardware demo, you will need physical hardware access. For instructions on how to set that up, please contact us. The software demo may be launched as follows. Note that these commands must be run in different terminal windows.
```
roslaunch meta_planner rviz.launch
roslaunch meta_planner software_demo.launch
```

To run unit tests, type:
```
catkin_make run_tests
```

## C++ reference materials
We attempt to adhere to the philosophy put forward in the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Our code is written _for the reader, not the writer_. We write comments liberally and use inheritance whenever it makes sense.

A few tips, tricks, and customs that you'll find throughout our code:
* Lines of code are no longer than 80 characters.
* The names of all member variables of a class end with an underscore, e.g. `foo_`.
* When iterating through a vector, we name the index something like `ii` instead of just `i`. This makes it super easy to find and replace the iterator later.
* We use the `const` specifier whenever possible.
* We try to include optional guard statements with meaningful debug messages wherever possible. These may be toggled on/off with the `ENABLE_DEBUG_MESSAGES` cmake option.
* Whenever it makes sense, we write unit tests for self-contained functionality and integration tests for dependent functions and classes. These are stored in the `test/` directory.
