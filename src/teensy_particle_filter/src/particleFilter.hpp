
#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>

#include "particle.hpp"
#include "motionModel.hpp"
#include "sensorModel.hpp"

#include "../config/mcl.h"

class ParticleFilter
{
private:
    std::vector<Particle> particles;
    
    MotionModel motionModel;
    SensorModel sensorModel;

    bool updating;


    geometry_msgs__msg__Pose latestOdom;
    geometry_msgs__msg__Pose previousOdom;
    sensor_msgs__msg__LaserScan latestLaserScan;

public:
    ParticleFilter();
    void initParticleFilter();
    void updateParticles();

    void updateLatestOdom(nav_msgs__msg__Odometry odom);
    void updatePreviousOdom(nav_msgs__msg__Odometry odom);
    void updateLatestLaserScan(sensor_msgs__msg__LaserScan laserScan);

    geometry_msgs__msg__Pose etimatePose();

    bool shouldResample();
    void resampleParticles();

};

#endif