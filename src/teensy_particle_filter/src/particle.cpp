#include "particle.hpp"

// Particle constructor
Particle::Particle()
{
    weight = 0.0;
    pose.orientation = {0,0,0,0};
    pose.position = {0,0,0};
}

// Date member function
// void Particle::SetDate(int year, int month, int day)
// {
//     m_month = month;
//     m_day = day;
//     m_year = year;
// }