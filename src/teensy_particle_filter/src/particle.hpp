#ifndef DATE_H
#define DATE_H

// Microros
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/pose.h>

#include <random>

// uncomment for DEAD TEENSY
// #include <cmath>
// #include <iostream>
#include "quat.hpp"

#define PI 3.1415926535897932384626433832795


class Particle
{
private:

public:

    float weight;
    geometry_msgs__msg__Pose pose;

    Particle();
    void setPose();
    void initParticle(double x_width,double y_width, int numParticles);
};

#endif