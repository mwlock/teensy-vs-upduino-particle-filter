
#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include <geometry_msgs/msg/pose.h>
#include <nav_msgs/msg/odometry.h>

#include <math.h>

#include "quat.hpp"

#define MOVED_TOO_CLOSE 0.01

class MotionModel
{
private:

public:

    MotionModel();

    // Sample motion model to predict particle movement
    static geometry_msgs__msg__Pose sampleMotionModel(
        geometry_msgs__msg__Pose previous_xt,
        geometry_msgs__msg__Pose latestOdom,
        geometry_msgs__msg__Pose prevOdom
    );

    // void SetDate(int year, int month, int day);

    // int getYear() { return m_year; }
    // int getMonth() { return m_month; }
    // int getDay()  { return m_day; }
};

#endif