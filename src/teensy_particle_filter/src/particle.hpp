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
#include <stdio.h>

#define PI 3.1415926535897932384626433832795


class Particle
{
private:

public:

    float weight;
    geometry_msgs__msg__Pose pose;

    Particle();
    void setPose();
    void initParticle(
        int numParticles,
        std::mt19937 generator,
        std::uniform_real_distribution<double> distribution_x,
        std::uniform_real_distribution<double> distribution_y,
        std::uniform_real_distribution<double> distribution_yaw
    );

    void initParticle(
        double x,
        double y,
        double yaw
    );

    void initParticle(
    double x,
    double y,
    double theta,
    double weight
    );
};

#endif