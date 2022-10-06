/**
* Very fast RNG. (for test purpose: no guarantee of good statistical properties). 
* The generator always use the same deterministic seed.
**/

// Taken from : https://forum.pjrc.com/threads/54113-Gaussian-Distributed-Random-Numbers?highlight=gaussian

// This is a very fast and simple Gaussian random number generator.

// Include uint32_t

#ifndef FastRNG_hpp
#define FastRNG_hpp

#include <stdint.h>
#include <random>
#include <math.h>
#include "quat.hpp"

class FastRNG
{
public:

    /* type of integer returned by the generator */
    typedef uint32_t result_type;

    /* Default constructor. Always initialize with the same seed. */
    FastRNG() : _gen_x(123456789), _gen_y(362436069), _gen_z(521288629) { }

    /* min value */
    static constexpr result_type min() { return 0; }

    /* max value */
    static constexpr result_type max() { return 4294967295UL; }

    /* return a random number */
    inline uint32_t operator()()
        {
        uint32_t t;
        _gen_x ^= _gen_x << 16; _gen_x ^= _gen_x >> 5; _gen_x ^= _gen_x << 1;
        t = _gen_x; _gen_x = _gen_y; _gen_y = _gen_z; _gen_z = t ^ _gen_x ^ _gen_y;
        return _gen_z;
        }

    /* return a uniform number on [0,1).  */
    inline float unif() { return ((float)(operator()())) / (4294967296.0); }

private:

    uint32_t _gen_x, _gen_y, _gen_z;        // state of the generator
};



// /* generate N(0,1) random variable using "ratio of uniform" method 
//    taken from "Numerical recipies in C++" 
//  */
// template<class random_t> inline float normalLaw(random_t & gen)
//     {
//     float u, v, x, y, q;
//     do {
//         u = gen.unif(); v = 1.7156*(gen.unif() - 0.5);
//         x = u - 0.449871; y = abs(v) + 0.386595;
//         q = (x*x) + y * (0.19600*y - 0.25472*x);
//         } 
//     while ((q > 0.27597) && (q > 0.27846 || (v*v) > -4.*logf(u)*(u*u)));
//     return (v / u);
//     }


/* generate N(0,1) random variable using "ratio of uniform" method 
   taken from "Numerical recipies in C++" 
 */
template<class random_t> inline float normalLaw(random_t & gen, double mean, double sigma)
    {
    float u, v, x, y, q;
    do {
        u = gen.unif(); v = 1.7156*(gen.unif() - 0.5);
        x = u - 0.449871; y = abs(v) + 0.386595;
        q = (x*x) + y * (0.19600*y - 0.25472*x);
        } 
    while ((q > 0.27597) && (q > 0.27846 || (v*v) > -4.*logf(u)*(u*u)));
    return (v / u) * sigma + mean;
    }

inline double normalDistribution(double mean, double sigma){

    /**
     * Generate a random number from a normal distribution
     * @param mean mean point around which to sample
     * @param std standard deviation of normal distribution
     */

    double u = (double)rand() / (double)RAND_MAX;
    double v = (double)rand() / (double)RAND_MAX;
    double x = sqrt(-2.0 * log(u)) * sin(2.0 * PI * v);
    return x * sigma + mean;
}


// /* test */
// void setup()
//     {
//     delay(1000);
//     while (!Serial) { yield(); }
//     const int32_t N = 1000000;
//     FastRNG gen;
//     float mean = 0.0, var = 0.0; 
//     elapsedMillis timer = 0;
//     for (int32_t i = 0; i < N; i++)
//         {
//         const float x = normalLaw(gen); 
//         mean += x;
//         var += x*x; 
//         }
//     const uint32_t t = timer;
//     Serial.print("mean : "); Serial.println(mean/N, 5);
//     Serial.print("standart dev: "); Serial.println(sqrt(var/(N-1)), 5);
//     Serial.print("generation took "); Serial.print(t); Serial.println("ms.");
//     }


// void loop()
// {

// }

#endif