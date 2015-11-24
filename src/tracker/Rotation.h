#ifndef _ROTATION_H_
#define _ROTATION_H_

#include <math.h>
#include <Eigen/Geometry>


class Rotation
{
public:
    Rotation() { setQuaternion(1., 0., 0., 0.); }
    Rotation(float w, float x, float y, float z) { setQuaternion(w, x, y, z); }
    Rotation(float roll, float pitch, float yaw) { setEuler(roll, pitch, yaw); }
    Rotation(Eigen::Vector3f v) { setEuler(v(0), v(1), v(2)); }
    
    ~Rotation() { }
    
    
    void setQuaternion(float w, float x, float y, float z)
    {
        q = Eigen::Quaternionf(w, x, y, z);
        q.normalize();
    }
    
    void setEuler(float roll, float pitch, float yaw)
    {
        float croll = cos(roll / 2.);
        float sroll = sin(roll / 2.);
        float cpitch = cos(pitch / 2.);
        float spitch = sin(pitch / 2.);
        float cyaw = cos(yaw / 2.);
        float syaw = sin(yaw / 2.);
        
        q.w() = croll * cpitch * cyaw + sroll * spitch * syaw;
        q.x() = sroll * cpitch * cyaw - croll * spitch * syaw;
        q.y() = croll * spitch * cyaw + sroll * cpitch * syaw;
        q.z() = croll * cpitch * syaw - sroll * spitch * cyaw;
        q.normalize();
    }

    float getRoll()
    {
        return atan2(2. * (q.w() * q.x() + q.y() * q.z()), 1. - 2. * (q.x() * q.x() + q.y() * q.y()));
    }
    
    float getPitch()
    {
        return asin(2. * (q.w() * q.y() - q.z() * q.x()));
    }
    
    float getYaw()
    {
        return atan2(2. * (q.w() * q.z() + q.x() * q.y()), 1. - 2. * (q.y() * q.y() + q.z() * q.z()));
    }
    
    Eigen::Quaternionf getQuaternion()
    {
        return q;
    }
    
    Eigen::Matrix3f getRotation()
    {
        Eigen::Matrix3f result;
        result << 1. - 2. * (q.y() * q.y() + q.z() * q.z()), 2. * (q.x() * q.y() - q.w() * q.z()), 2. * (q.w() * q.y() + q.x() * q.z()),
            2. * (q.x() * q.y() + q.w() * q.z()), 1. - 2. * (q.x() * q.x() + q.z() * q.z()), 2. * (q.y() * q.z() - q.w() * q.x()),
            2. * (q.x() * q.z() - q.w() * q.y()), 2. * (q.w() * q.x() + q.y() * q.z()), 1. - 2. * (q.x() * q.x() + q.y() * q.y());
        
        return result;
    }

private:
    Eigen::Quaternion<float> q;
};

#endif /* _ROTATION_H_ */
