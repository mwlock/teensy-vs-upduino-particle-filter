#include "particleFilter.hpp"
#include "particle.hpp"

ParticleFilter::ParticleFilter(){
    updating = false;
    motionModel = MotionModel();
    sensorModel = SensorModel();

    latestOdom = geometry_msgs__msg__Pose();
    previousOdom = geometry_msgs__msg__Pose();
    latestLaserScan = sensor_msgs__msg__LaserScan();
}

void ParticleFilter::initParticleFilter(){

    // Iterate through particles and initialise particles
    for (int i = 0; i < NUM_OF_PARTICLES; i++) {
        Particle particle = Particle();                 // Create new particle
        particle.initParticle(3,3,NUM_OF_PARTICLES);    // Init the particles
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
        geometry_msgs__msg__Pose predictedPose = motionModel.sampleMotionModel(
            currentPose,
            latestOdom,
            previousOdom
        );  // predict the pose using the motion model

        // Determine the weight of the particle
        double weight = sensorModel.sampleSensorModel(predictedPose, latestLaserScan);

        // Create new particle
        Particle newParticle = Particle();
        newParticle.pose = predictedPose;
        newParticle.weight = weight;

        // Add the particle to the new particles list
        newParticles.push_back(newParticle);

    }

    // Normalise the weights of the new particles
    double totalWeight = 0;
    for(Particle particle : newParticles){
        totalWeight += particle.weight;
    }
    for(Particle particle : newParticles){
        particle.weight = particle.weight / totalWeight;
    }

    // Clear the current particles
    particles.clear();

    // Copy the new particles to the current particles
    for(Particle particle : newParticles){
        particles.push_back(particle);
    }

    // Estimate the pose of the robot
    geometry_msgs__msg__Pose estimatedPose = etimatePose();

    //TODO Publish new path


    // Resmaple the particles if needed
    if(shouldResample()){
        resampleParticles();
    }

    // Update the particles 
    updating = false;

}

void ParticleFilter::updateLatestOdom(nav_msgs__msg__Odometry odom){

    // Update the latest odom
    if (!updating){
        latestOdom = odom.pose.pose;
    }
}

void ParticleFilter::updatePreviousOdom(nav_msgs__msg__Odometry odom){
    // Update the previous odom
    previousOdom = odom.pose.pose;
}

void ParticleFilter::updateLatestLaserScan(sensor_msgs__msg__LaserScan laserScan){
    // Update the latest laser scan
    if (!updating){
        latestLaserScan = laserScan;
    }
}

geometry_msgs__msg__Pose ParticleFilter::etimatePose(){

    // Get the estimated pose
    SimplePose estimatedPose = SimplePose();
    estimatedPose.x = 0;
    estimatedPose.y = 0;
    estimatedPose.theta = 0;

    // Iterate through the particles and get the average pose
    for(Particle particle : particles){
        estimatedPose.x += particle.pose.position.x * particle.weight;
        estimatedPose.y += particle.pose.position.y * particle.weight;
        Quaternion q = {particle.pose.orientation.w, particle.pose.orientation.x, particle.pose.orientation.y, particle.pose.orientation.z};
        double particleTheta = Quat::ToEulerAngles(q).yaw;
        estimatedPose.theta += particleTheta* particle.weight;
    }

    // Convert the estimated pose to a geometry_msgs::Pose
    geometry_msgs__msg__Pose estimatedPoseMsg = geometry_msgs__msg__Pose();
    estimatedPoseMsg.position.x = estimatedPose.x;
    estimatedPoseMsg.position.y = estimatedPose.y;
    estimatedPoseMsg.position.z = 0;    
    EulerAngles angles = {0, 0, estimatedPose.theta};
    Quaternion q = Quat::EulerToQuaternion(angles);
    estimatedPoseMsg.orientation.w = q.w;
    estimatedPoseMsg.orientation.x = q.x;
    estimatedPoseMsg.orientation.y = q.y;
    estimatedPoseMsg.orientation.z = q.z;

    return estimatedPoseMsg;

}

bool ParticleFilter::shouldResample(){

    double resample_threshold = NUM_OF_PARTICLES/5;

    // Determine if resampling is required using particle weights squared
    double totalWeight = 0;
    for(Particle particle : particles){
        totalWeight += particle.weight * particle.weight;
    }

    totalWeight = totalWeight / NUM_OF_PARTICLES;

    // If the total weight is less than the threshold, resample
    return 1/totalWeight < resample_threshold;

}

// Resample particles using CUMSUM
void ParticleFilter::resampleParticles(){

    // Get the cumulative sum of the weights
    std::vector<double> cumsum;
    double totalWeight = 0;
    for(Particle particle : particles){
        totalWeight += particle.weight;
        cumsum.push_back(totalWeight);
    }

    // Create a new set of particles
    std::vector<Particle> newParticles;

    // Iterate through the particles and resample
    for(int i = 0; i < NUM_OF_PARTICLES; i++){

        // Get a random number between 0 and 1
        double r = (double)rand() / (double)RAND_MAX;

        // Find the index of the particle to resample
        int index = 0;
        for(int j = 0; j < NUM_OF_PARTICLES; j++){
            if(cumsum[j] > r){
                index = j;
                break;
            }
        }

        // Add the particle to the new particles list
        newParticles.push_back(particles[index]);

    }

    // Clear the current particles
    particles.clear();

    // Copy the new particles to the current particles
    for(Particle particle : newParticles){
        particles.push_back(particle);
    }

}