#include "particleFilter.hpp"
#include "particle.hpp"

extern int clock_gettime(clockid_t unused, struct timespec *tp);

ParticleFilter::ParticleFilter(void (*callback)(const char*))
{
    this->printDebug = callback;

    updating = false;
    motionModel = MotionModel();
    sensorModel = SensorModel();

    latestOdom = geometry_msgs__msg__Pose();
    previousOdom = geometry_msgs__msg__Pose();
    latestLaserScan = sensor_msgs__msg__LaserScan();

    // Create pose array (malloc once to prevent memory leak)
    this->poseArray = geometry_msgs__msg__PoseArray();
    bool success = rosidl_runtime_c__String__assign(&poseArray.header.frame_id,"map");
    poseArray.poses.size = NUM_OF_PARTICLES;
    poseArray.poses.capacity = NUM_OF_PARTICLES;
    poseArray.poses.data = (geometry_msgs__msg__Pose*)malloc(NUM_OF_PARTICLES * sizeof(geometry_msgs__msg__Pose));

    // Initialise last odoms
    lastUsedOdomInitialised = false;
    lastOdomInitialised = false;
    particleFilterInitialised = false;

}

ParticleFilter::ParticleFilter(){
    updating = false;
    motionModel = MotionModel();
    sensorModel = SensorModel();

    latestOdom = geometry_msgs__msg__Pose();
    previousOdom = geometry_msgs__msg__Pose();
    latestLaserScan = sensor_msgs__msg__LaserScan();

    // Create pose array (malloc once to prevent memory leak)
    geometry_msgs__msg__PoseArray poseArray = geometry_msgs__msg__PoseArray();
    bool success = rosidl_runtime_c__String__assign(&poseArray.header.frame_id,"map");
    poseArray.poses.size = NUM_OF_PARTICLES;
    poseArray.poses.capacity = NUM_OF_PARTICLES;
    poseArray.poses.data = (geometry_msgs__msg__Pose*)malloc(NUM_OF_PARTICLES * sizeof(geometry_msgs__msg__Pose));

    // Initialise last odoms
    lastUsedOdomInitialised = false;
    lastOdomInitialised = false;
}

void ParticleFilter::initParticleFilter(){

    const double X_WIDTH = 3;
    const double Y_WIDTH = 3;
    const double YAW_WIDTH = PI;

    this->printDebug("initalising particles in function");  

    // Iterate through particles and initialise particles
    for (int i = 0; i < NUM_OF_PARTICLES; i++) {

        Particle particle = Particle();                                                     // Create new particle

        double x = (double)rand()/(double)RAND_MAX * X_WIDTH - X_WIDTH/2;
        double y = (double)rand()/(double)RAND_MAX * Y_WIDTH - Y_WIDTH/2;
        double yaw = (double)rand()/(double)RAND_MAX * YAW_WIDTH - YAW_WIDTH/2;
        
        particle.initParticle(x, y, yaw, 1.0/NUM_OF_PARTICLES);                             // Init the particles
        particles.push_back(particle);                                                      // Append particle to the vector (dynamic list)

        // Print x and y coordinates of particle i 
        char buffer[100];
        sprintf(buffer, "NEW Particle %d: x = %f, y = %f, weight = %f", i, particles.at(i).pose.position.x, particles.at(i).pose.position.y, particles.at(i).weight);
        this->printDebug(buffer);

    }

    // Set particle filter as initialised
    particleFilterInitialised = true;

}

void ParticleFilter::updateParticles(){

    // Check if new particle data came in 
    if (Quat::arePosesEqual(latestOdom, previousOdom)) {
        this->printDebug("No new data");
        return;
    }

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
    // geometry_msgs__msg__Pose estimatedPose = etimatePose();

    //TODO Publish new path


    // Resmaple the particles if needed
    if(shouldResample()){
        // resampleParticles();
    }

    // Update the particles 
    updating = false;

}

void ParticleFilter::updateLatestOdom(nav_msgs__msg__Odometry odom){

    // Set initial odom
    if(!lastOdomInitialised){
        lastOdomInitialised = true;
        lastUsedOdomInitialised = true;
    }

    // Update the latest odom
    if (!updating){
        // Update latestOdom field by field
        latestOdom.position.x = odom.pose.pose.position.x;
        latestOdom.position.y = odom.pose.pose.position.y;
        latestOdom.position.z = odom.pose.pose.position.z;
        latestOdom.orientation.x = odom.pose.pose.orientation.x;
        latestOdom.orientation.y = odom.pose.pose.orientation.y;
        latestOdom.orientation.z = odom.pose.pose.orientation.z;
        latestOdom.orientation.w = odom.pose.pose.orientation.w;
    }
}

void ParticleFilter::updatePreviousOdom(){
    // Copy each field of odom to previous odom
    previousOdom.position.x = this->latestOdom.position.x;
    previousOdom.position.y = this->latestOdom.position.y;
    previousOdom.position.z = this->latestOdom.position.z;
    previousOdom.orientation.x = this->latestOdom.orientation.x;
    previousOdom.orientation.y = this->latestOdom.orientation.y;
    previousOdom.orientation.z = this->latestOdom.orientation.z;
    previousOdom.orientation.w = this->latestOdom.orientation.w;
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
        double particleTheta = Quat::yawFromPose(particle.pose);
        estimatedPose.theta += particleTheta * particle.weight;
    }

    // Convert the estimated pose to a geometry_msgs::Pose
    geometry_msgs__msg__Pose estimatedPoseMsg = geometry_msgs__msg__Pose();
    estimatedPoseMsg = Quat::poseFromXYZRPY(estimatedPose.x, estimatedPose.y, 0, 0, 0, estimatedPose.theta);

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

// Return array of poses
geometry_msgs__msg__PoseArray ParticleFilter::getPoseArray(){

    // https://answers.ros.org/question/381123/how-to-generate-header-stamp-rpi-pico/
    // https://github.com/micro-ROS/freertos_apps/issues/62

    // Iterate through the particles and add them to the pose array
    for(int i = 0; i < NUM_OF_PARTICLES; i++){
        poseArray.poses.data[i] = particles.at(i).pose;
    }

    // Fill the message timestamp
    // https://github.com/micro-ROS/micro_ros_arduino/issues/1122
    poseArray.header.stamp.sec = (u_int16_t)(rmw_uros_epoch_millis()/1000);
    poseArray.header.stamp.nanosec = (u_int32_t)rmw_uros_epoch_nanos();

    return poseArray;
}


bool ParticleFilter::isInitialised(){

    // Check if the particle filter is initialised and odom message has been recieved
    return (lastUsedOdomInitialised && lastOdomInitialised && particleFilterInitialised);

}
 bool ParticleFilter::isParticlesInitialised(){

    // Check if the particles have been initalised
    return particleFilterInitialised;

 }

