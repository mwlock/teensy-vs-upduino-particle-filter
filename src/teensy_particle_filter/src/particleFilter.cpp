#include "particleFilter.hpp"
#include "particle.hpp"

ParticleFilter::ParticleFilter(){
    updating = false;
}

void ParticleFilter::initParticleFilter(){

    // Iterate through particles and initialise particles
    for (int i = 0; i < num_of_particles; i++) {
        Particle particle = Particle();                 // Create new particle
        particle.initParticle(3,3,num_of_particles);    // Init the particles
        particles.push_back(particle);                  // Append particle to the vector (dynamic list)
    }

}

void ParticleFilter::updateParticles(){

    updating = true;

    // New particles
    std::vector<Particle> newParticles;

    // Iterate through current particles
    for(Particle particle : particles){
        
        geometry_msgs__msg__Pose currentPose = particle.pose;
        geometry_msgs__msg__Pose predictedPose = particle.pose; // predict the pose using the motion model

    } 

    updating = false;

}