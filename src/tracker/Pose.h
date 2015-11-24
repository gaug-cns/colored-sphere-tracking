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
    
    Pose operator+(const Pose &motion)
    {
        Pose result = Pose();
        Rotation rot = Rotation(orientation);
        result.position = position + rot.getRotation() * motion.position;
        result.orientation = orientation + motion.orientation;
        result.time = time + motion.time;
        return result;
    }
    
    Pose operator-(const Pose &motion)
    {
        Pose result = Pose();
        Rotation rot = Rotation(-orientation);
        result.position = position - rot.getRotation() * motion.position;
        result.orientation = orientation - motion.orientation;
        result.time = time - motion.time;
        return result;
    }
    
    Pose operator+=(const Pose &motion)
    {
        return *this + motion;
    }
    
    void rotate(float roll, float pitch, float yaw)
    {
        Rotation rot = Rotation(roll, pitch, yaw);
        position = rot.getRotation() * position;
        orientation = rot.getRotation() * orientation;
    }
};

#endif /* _POSE_H_ */
