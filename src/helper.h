#ifndef _HELPER_H_
#define _HELPER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <map>
#include <sys/stat.h>
#include <math.h>

#include <Eigen/Dense>
#include "opencv2/opencv.hpp"


#define PI 3.1415
#define SQUARE(a) (pow(a, 2))


#define WIDTH 640 // [px]
#define HEIGHT 480 // [px]
#define BALL_RADIUS 0.03 // [m]
#define CAMERA_X_ANGLE 44.5 * PI / 180. // [rad], 0 rad -> horizontal, pi/2 -> vertical down
#define CAMERA_F 525. // a.u.


enum Color
{
	Red,
	Green,
	Yellow,
	Blue
};

struct Measurement
{
    Color color;
    
    Eigen::Vector3f mean;
    float sigma;
    float probability;
    float time;
    
    float maximum;
    
    float time_factor(float T)
    {
        return 1. / (1 + exp(0.5 * (T - time - 1 - 0.5)));
    }
    
    void calculate(float T)
    {
        maximum = probability * time_factor(T) * pow(2 * PI * SQUARE(sigma), -0.5);
    }
    
    Measurement(Eigen::Vector3f mean, Color color) : mean(mean), color(color) { }
    
    bool operator < (const Measurement& m) const
    {
        return (maximum < m.maximum);
    }
};

struct Sphere
{
    Color color;
    Eigen::Vector3f position;
	
	Sphere(Eigen::Vector3f position, Color color) : position(position), color(color) {};
    Sphere(float x, float y, float z, Color color) : color(color) {
        position << x, y, z;
    };
};

struct Pose
{
    Eigen::Vector3f position;
    Eigen::Vector3f orientation;
    int time;
    
    Pose operator-(const Pose &r)
    {
        Pose result = {
            position - r.position,
            orientation - r.orientation,
            time - r.time
        };
        return result;
    }
};



#define BACKGROUND_IMAGE_DIR "../calibration/background.jpg"
#define DEPTH_BACKGROUND_IMAGE_DIR "../calibration/depth_background.jpg"
#define SAVE_IMAGE_DIR "../calibration/shot.jpg"
#define TRACKING_DATA_DIR "../tracking_data.txt"

int getTime()
{
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> ( std::chrono::system_clock::now().time_since_epoch() );
    return ms.count();
}

#endif