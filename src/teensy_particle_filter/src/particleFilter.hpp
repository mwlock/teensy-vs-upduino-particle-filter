
#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>

#include "particle.hpp"
#include "../config/mcl.h"

class ParticleFilter
{
private:
    std::vector<Particle> particles;
    bool updating;

public:
    ParticleFilter();
    void initParticleFilter();
    void updateParticles();

    // void SetDate(int year, int month, int day);

    // int getYear() { return m_year; }
    // int getMonth() { return m_month; }
    // int getDay()  { return m_day; }
};

#endif