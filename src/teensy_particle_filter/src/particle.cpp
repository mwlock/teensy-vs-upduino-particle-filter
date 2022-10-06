#include "particle.hpp"

// Particle constructor
Particle::Particle()
{   
    // Initialise the particle
    weight = 0.0;
    pose.orientation = {0,0,0,0};
    pose.position = {0,0,0};

}

void Particle::initParticle(
    int numParticles,
    std::mt19937 generator,
    std::uniform_real_distribution<double> distribution_x,
    std::uniform_real_distribution<double> distribution_y,
    std::uniform_real_distribution<double> distribution_yaw
    ){
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

    // Init pose
    weight = 1/double(numParticles);
    pose = Quat::poseFromXYZRPY(distribution_x(generator), 
                                distribution_y(generator), 
                                0, 
                                0, 0, distribution_yaw(generator));
}

void Particle::initParticle(
    double x,
    double y,
    double weight
    ){
    /**
     * Ititalise pose of a partile
     * 
     *
     * @param x x position of particle
     * @param y y position of particle
     * @param weight weight of particle
     */

    // Init pose
    this->weight = weight;
    pose.position.x = x;
    pose.position.y = y;
    EulerAngles angle = {0,0,0};
    Quaternion q = Quat::EulerToQuaternion(angle);
    pose.orientation.x = q.x;
    pose.orientation.y = q.y;
    pose.orientation.z = q.z;
    pose.orientation.w = q.w;
}

void Particle::initParticle(
    double x,
    double y,
    double yaw,
    double weight
    ){
    /**
     * Ititalise pose of a partile
     * 
     *
     * @param x x position of particle
     * @param y y position of particle
     * @param yaw yaw of particle
     */

    // Init pose
    this->weight = weight;
    this->pose = Quat::poseFromXYZRPY(x, y, 0, 0, 0, yaw);
}