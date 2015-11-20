# Colored Sphere Tracking

This software uses a Kinect (or Asus Xtion Pro) and colored spheres to estimate the pose of an object. 

The algorithm processes following steps:
- Background subtraction (in /calibration directory, a new background is taken with `p`)
- Detection of colored regions using a HSV color filter
- Detection of spheres with Hough circles algorithm
- Error correction using the depth and hough radius and known sphere size
- Likelihood filter for better estimation
- Pose calculation from a number of detected sphere positions.



## Installation

Colored Sphere Tracking depends on:
- [OpenNI 2](http://structure.io/openni)
- [OpenCV](http://opencv.org) (with OpenNI 2 support, this needs to be enabled in build step)
- [Eigen](http://eigen.tuxfamily.org)
- [Ceres Solver](http://ceres-solver.org)

Colored Sphere Tracking uses [CMake](http://www.cmake.org) for building, therefore run:
1. `mkdir build && cd build`
2. `cmake ..``
3. `make`



## Contribution

Colored Sphere Tracking follows the [ROS C++ Coding Style](http://wiki.ros.org/CppStyleGuide).



## License

Colored Sphere Tracking is released with a BSD license. For full terms and conditions, see the LICENSE file.