#ifndef DATE_H
#define DATE_H

#include <geometry_msgs/msg/pose.h>
#include <random>

#define M_PI 3.1415926535897932384626433832795


class Particle
{
private:
    float weight;
    geometry_msgs__msg__Pose pose;
    float a = M_PI;

public:
    Particle();

    void setPose();
    void initParticle(double x_width,double y_width, int numParticles);
};

#endif