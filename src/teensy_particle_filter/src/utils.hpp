
#ifndef UTILS_H
#define UTILS_H

#include "particle.hpp"

class Utils
{
private:
    float weight;
    geometry_msgs__msg__Pose pose;

public:
    Utils();

    void initializParticle(Particle* particle, double x_width,double y_width);



    // void SetDate(int year, int month, int day);

    // int getYear() { return m_year; }
    // int getMonth() { return m_month; }
    // int getDay()  { return m_day; }
};

#endif