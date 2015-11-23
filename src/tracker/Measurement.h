#ifndef _MEASUREMENT_H_
#define _MEASUREMENT_H_

#include <math.h>

#include <Eigen/Dense>



struct Measurement
{
    Measurement(Eigen::Vector3f mean, Color color) : mean(mean), color(color) { }
    
    float time_factor(float T)
    {
        return 1. / (1 + exp(0.5 * (T - time - 1 - 0.5)));
    }
    
    void calculate(float T)
    {
        maximum = probability * time_factor(T) * pow(2 * M_PI * pow(sigma, 2), -0.5);
    }
    
    bool operator < (const Measurement& m) const
    {
        return (maximum < m.maximum);
    }
    
    
    Color color;
    
    Eigen::Vector3f mean;
    float sigma;
    float probability;
    float time;
    
    float maximum;
};

#endif /* _MEASUREMENT_H_ */
