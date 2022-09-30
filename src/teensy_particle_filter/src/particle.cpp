#include "particle.hpp"

// Get mathematical constants
#include <cmath>
#include <iostream>
#define _USE_MATH_DEFINES

// Particle constructor
Particle::Particle()
{
    weight = 0.0;
    pose.orientation = {0,0,0,0};
    pose.position = {0,0,0};
}

void Particle::initParticle(double x_width,double y_width, int numParticles){
    /**
     * Ititalise pose of a partile
     * https://cplusplus.com/reference/random/normal_distribution/
     * https://cplusplus.com/reference/random/uniform_real_distribution/
     *
     * @param particle fpointer to the particle we want to initialise
     * @param lowerBound x_width of map to sample particle from using uniform distribution
     * @param upperBound y_width of map to sample particle from using uniform distribution
     * @param numPartciles number of particles being generated
     */

    // std::default_random_engine generator;
    // std::uniform_real_distribution<double> distribution_x(-x_width/2,x_width/2);
    // std::uniform_real_distribution<double> distribution_y(-y_width/2,y_width/2);
    // std::uniform_real_distribution<double> distribution_yaw(-PI/2,PI/2);

    // // Init pose
    // weight = 1/numParticles;
    // pose.position.x = distribution_x(generator);
    // pose.position.y = distribution_y(generator);
    // pose.orientation.z = 0;

}