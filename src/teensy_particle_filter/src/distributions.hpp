#ifndef DISTRIBUTIONS_H
#define DISTRIBUTIONS_H


#include <random>

class Distributions
{
private:
    /* data */
public:
    Distributions();
    ~Distributions();
    static double sampleNormal(double mean, double std);
};

#endif