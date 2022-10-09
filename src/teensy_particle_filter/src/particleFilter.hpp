
#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>

#include "../include/map_array.h"
#include <tuple>
#include "particle.hpp"
#include "motionModel.hpp"
#include "sensorModel.hpp"
#include "FastRNG.hpp"

#include "../config/mcl.h"

#include <geometry_msgs/msg/pose_array.h>
#include "rosidl_runtime_c/string_functions.h"
#include "geometry_msgs/msg/detail/pose_array__functions.h"
#include <micro_ros_platformio.h>

#include <rcl/error_handling.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <nav_msgs/msg/odometry.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <exception>

// Random number generator
#include <Entropy.h>
#include <random>

// Microros
#include <micro_ros_utilities/type_utilities.h>

#define ALMOST_ZERO 1e-15

class ParticleFilter
{
private:
    std::vector<Particle> particles;
    std::vector<std::tuple<double, double>> map_obstacles;
    
    MotionModel motionModel;
    SensorModel sensorModel;

    bool updating;

    geometry_msgs__msg__Pose latestOdom;
    geometry_msgs__msg__Pose previousOdom;
    sensor_msgs__msg__LaserScan latestLaserScan;
    geometry_msgs__msg__PoseArray poseArray;

    geometry_msgs__msg__Pose prev_estimated_pose;
    geometry_msgs__msg__PoseArray prev_pose_array;

    bool lastUsedOdomInitialised;
    bool lastOdomInitialised;
    bool particleFilterInitialised;
    bool laserScanUpdating;

    void (*printDebug)(const char*);

public:

    // Constructor that takes in function pointer as an argument
    ParticleFilter(void (*callback)(const char*));

    ParticleFilter();
    void initParticleFilter();
    std::vector<std::tuple<double, double>> getMapObstacles();
    std::tuple<geometry_msgs__msg__PoseArray, geometry_msgs__msg__Pose, bool> updateParticles();

    void updateLatestOdom(nav_msgs__msg__Odometry odom);
    void updatePreviousOdom();
    void updateLatestLaserScan(sensor_msgs__msg__LaserScan laserScan);

    geometry_msgs__msg__Pose etimatePose();
    geometry_msgs__msg__PoseArray getPoseArray();

    bool shouldResample();
    void resampleParticles();
    bool isUpdating();

    // Check if odom is initalised 
    bool isInitialised();
    bool isParticlesInitialised();
    bool odomWasUpdated();
};

#endif