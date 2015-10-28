# Colored Sphere Tracking

This software uses a Kinect (or Asus Xtion Pro) and colored spheres to estimate the pose of a robot. 

Following steps are roughly used for tracking:
- Background subtraction (in /calibration directory, a new background is taken with 'p')
- Detection of colored regions using a HSV color filter
- Detection of spheres with Hough circles algorithm
- Error correction using the depth and hough radius and known sphere size
- Likelihood filter
- Pose calculation from a number of detected sphere positions.



## Installation

Colored Sphere Tracking depends on:
- Eigen3
- OpenNI2
- OpenCV 3 (with OpenNI2 support, can be enabled in build step)

It is compiled with `CMake`, running:
- mkdir build && cd build
- cmake ..
- make



## Development

[ROS C++ Coding Style](http://wiki.ros.org/CppStyleGuide)



## License

Colored Sphere Tracking is released with a BSD license. For full terms and conditions, see the LICENSE file.