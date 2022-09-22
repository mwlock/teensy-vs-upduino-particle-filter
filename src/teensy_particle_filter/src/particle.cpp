#include "particle.hpp"

// Date constructor
Particle::Particle(int year, int month, int day)
{
    SetDate(year, month, day);
}

// Date member function
void Particle::SetDate(int year, int month, int day)
{
    m_month = month;
    m_day = day;
    m_year = year;
}