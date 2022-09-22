#ifndef DATE_H
#define DATE_H

#include <geometry_msgs/msg/pose.h>

class Particle
{
private:
    float weight;
    geometry_msgs__msg__Pose pose;

public:
    Particle();

    // void SetDate(int year, int month, int day);

    // int getYear() { return m_year; }
    // int getMonth() { return m_month; }
    // int getDay()  { return m_day; }
};

#endif