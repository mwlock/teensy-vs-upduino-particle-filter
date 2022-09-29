#include "utils.hpp"
#include <random>

void Utils::initializParticle(Particle* particle, double x_width,double y_width){
    /**
     * Ititalise pose of a partile
     * https://cplusplus.com/reference/random/normal_distribution/
     * https://cplusplus.com/reference/random/uniform_real_distribution/
     *
     * @param particle fpointer to the particle we want to initialise
     * @param lowerBound x_width of map to sample particle from using uniform distribution
     * @param upperBound y_width of map to sample particle from using uniform distribution
     */

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution_x(-x_width/2,x_width/2);
    std::uniform_real_distribution<double> distribution_y(-y_width/2,y_width/2);   

}