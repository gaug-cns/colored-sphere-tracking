#ifndef _HELPER_H_
#define _HELPER_H_

#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <math.h>
#include <vector>
#include <map>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


#define WIDTH 640 // [px]
#define HEIGHT 480 // [px]
#define BALL_RADIUS 0.03 // [m]
#define CAMERA_X_ANGLE 44.5 * M_PI / 180. // [rad], 0 rad -> horizontal, pi/2 -> vertical down
#define CAMERA_F 525. // a.u.

#define BACKGROUND_IMAGE_DIR "../calibration/background.jpg"
#define DEPTH_BACKGROUND_IMAGE_DIR "../calibration/depth_background.jpg"
#define SAVE_IMAGE_DIR "../calibration/shot.jpg"
#define TRACKING_DATA_DIR "../tracking_data.txt"


enum Color
{
	Red,
	Green,
	Yellow,
	Blue
};


double getTime()
{
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> ( std::chrono::system_clock::now().time_since_epoch() );
    return ((double)ms.count()) / 1000;
}

#endif /* _HELPER_H_ */
