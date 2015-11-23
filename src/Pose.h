#ifndef _POSE_H_
#define _POSE_H_

#include <Eigen/Dense>


#define X 0
#define Y 1
#define Z 2
#define ROLL 0
#define PITCH 1
#define YAW 2

struct Pose
{
    Eigen::Vector3f position;
    Eigen::Vector3f orientation;
    double time;
    
    Pose() : time(0.)
    {
        position = Eigen::Vector3f::Zero();
        orientation = Eigen::Vector3f::Zero();
    }
    
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

#endif /* _POSE_H_ */
