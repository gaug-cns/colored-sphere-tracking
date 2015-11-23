#ifndef _SPHERE_H_
#define _SPHERE_H_

#include <Eigen/Dense>
#include "Helper.h"



struct Sphere
{
    Color color;
    Eigen::Vector3f position;
	
	Sphere(Eigen::Vector3f position, Color color) : position(position), color(color) {};
    
    Sphere(float x, float y, float z, Color color) : color(color)
    {
        position << x, y, z;
    };
};

#endif /* _SPHERE_H_ */
