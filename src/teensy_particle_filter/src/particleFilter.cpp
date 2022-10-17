#include "particleFilter.hpp"
#include "particle.hpp"

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

ParticleFilter::ParticleFilter(void (*callback)(const char*))
{
    this->printDebug = callback;

    // Get a vector of all the map points containing an obstacle
    this->map_obstacles = this->getMapObstacles();

    updating = false;
    motionModel = MotionModel();
    sensorModel = SensorModel(this->map_obstacles);

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

    // Allocate memory for latestLaserScan
    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 21;
    conf.max_basic_type_sequence_capacity = 21;
    success = micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        &latestLaserScan,
        conf
    );

}

ParticleFilter::ParticleFilter(){
    // Get a vector of all the map points containing an obstacle
    this->map_obstacles = this->getMapObstacles();
    updating = false;
    this->laserScanUpdating = false;
    motionModel = MotionModel();
    sensorModel = SensorModel(this->map_obstacles);

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

std::vector<std::tuple<double, double>> ParticleFilter::getMapObstacles(){
    int MAP_HEIGHT =  sizeof(map_array) / sizeof(map_array[0]);
    int MAP_WIDTH = sizeof(map_array[0]) / sizeof(bool);
    std::vector<std::tuple<double, double>> map_obstacles;
    for (int i = 0; i < MAP_HEIGHT; i++){
        for (int j = 0; j < MAP_WIDTH; j++){
            if (map_array[i][j] == 1){
                double y_pos = (MAP_HEIGHT - i) * MAP_RESOLUTION;
                double x_pos = j * MAP_RESOLUTION;
                map_obstacles.push_back(std::make_tuple(x_pos, y_pos));
            }
        }
    }
    return map_obstacles;
}

void ParticleFilter::initParticleFilter(){

    // const double X_WIDTH = 1.5;
    // const double Y_WIDTH = 1.5;
    // const double YAW_WIDTH = PI;

    this->printDebug("initalising particles in function");  

    // Iterate through particles and initialise particles
    for (int i = 0; i < NUM_OF_PARTICLES; i++) {

        Particle particle = Particle();                                                     // Create new particle

        // Initialise particle position using normal distribution around center position
        double x = normalDistribution(0, 0.05);
        double y = normalDistribution(0, 0.05);
        double yaw = normalDistribution(0, 0.01);

        // double x = (double)rand()/(double)RAND_MAX * X_WIDTH - X_WIDTH/2;
        // double y = (double)rand()/(double)RAND_MAX * Y_WIDTH - Y_WIDTH/2;
        // double yaw = (double)rand()/(double)RAND_MAX * YAW_WIDTH - YAW_WIDTH/2;
        
        particle.initParticle(x, y, yaw, 1.0/NUM_OF_PARTICLES);                             // Init the particles
        particles.push_back(particle);                                                      // Append particle to the vector (dynamic list)
        
    }

    // Set particle filter as initialised
    particleFilterInitialised = true;

}

bool ParticleFilter::odomWasUpdated(){
    if (Quat::arePosesEqual(this->latestOdom, this->previousOdom) || this->laserScanUpdating){
        return false;
    }
    return true;
}

std::tuple<geometry_msgs__msg__PoseArray,geometry_msgs__msg__Pose,bool> ParticleFilter::updateParticles(){

    // Check if new particle data came in 
    if (!this->odomWasUpdated()){
        this->printDebug("No new data");
        return std::make_tuple(this->prev_pose_array, this->prev_estimated_pose,false);
    }

    updating = true;

    // New particles
    std::vector<Particle> newParticles;

    // Check if particle acceleration is enabled
    #ifdef USE_HARDWARE_ACCELERATION

        float scans[NUM_LASERS];
        uint16_t projectedPoints[NUM_OF_PARTICLES*NUM_LASERS][2];

        // Get laser scans from latest laser scans
        for (int i = 0; i < NUM_LASERS; i++){
            if(!(range < laserScan.range_min || range > laserScan.range_max)){  
                scans[i] = this->latestLaserScan.ranges.data[i];
            }
        }
        for (Particle particle : particles){

            geometry_msgs__msg__Pose currentPose = particle.pose;
            geometry_msgs__msg__Pose predictedPose = motionModel.sampleMotionModel(
                currentPose,
                latestOdom,
                previousOdom,
                this->printDebug
            );  // predict the pose using the motion model

            // Create new particle
            Particle newParticle = Particle();
            newParticle.pose = predictedPose;
            newParticle.weight = particle.weight;
            newParticles.push_back(newParticle);

            // Project the particle onto the map grid
            SimplePose mapPose = SensorModel::calculateMapPose(predictedPose);
            float x = mapPose.x + laserScan.ranges.data[i] * cos(angle + mapPose.theta);
            float y = mapPose.y + laserScan.ranges.data[i] * sin(angle + mapPose.theta);
            calculateGridPose(x, y, &projectedPoints[i][0], &projectedPoints[i][1]);
        }

        // Calculate the 

        
    #else
    // ==========================================================================================
    // Iterate through current particles
    // ==========================================================================================
    for(Particle particle : particles){
        geometry_msgs__msg__Pose currentPose = particle.pose;
        geometry_msgs__msg__Pose predictedPose = motionModel.sampleMotionModel(
            currentPose,
            latestOdom,
            previousOdom,
            this->printDebug
        );  // predict the pose using the motion model

        // Determine the weight of the particle
        double weight = sensorModel.sampleSensorModel(predictedPose, latestLaserScan, printDebug);

        // Create new particle
        Particle newParticle = Particle();
        newParticle.pose = predictedPose;
        newParticle.weight = weight;

        // Add the particle to the new particles list
        newParticles.push_back(newParticle);

    }
    // ==========================================================================================
    #endif

    // Normalise the weights of the new particles
    double totalWeight = 0;
    
    // Iterate through new particles using index
    for (int i = 0; i < NUM_OF_PARTICLES; i++) {
        totalWeight += newParticles.at(i).weight;
    }
    // Iterate through new particles using index
    if (totalWeight > ALMOST_ZERO) {
        for (int i = 0; i < NUM_OF_PARTICLES; i++) {
            newParticles.at(i).weight /= totalWeight;
        }
    } else {
        for (int i = 0; i < NUM_OF_PARTICLES; i++) {
            newParticles.at(i).weight = 1.0/NUM_OF_PARTICLES;
        }
    }
    

    // Clear the current particles
    this->particles.clear();

    // Copy the new particles to the current particles
    for(Particle particle : newParticles){
        this->particles.push_back(particle);
    }


    // Update the particles 
    geometry_msgs__msg__PoseArray pa = this->getPoseArray();
    geometry_msgs__msg__Pose est_pose = this->etimatePose();


    std::tuple<geometry_msgs__msg__PoseArray,geometry_msgs__msg__Pose,bool> result = std::make_tuple(pa, est_pose,true);

    // Update previous estimated pose and pose array
    this->prev_estimated_pose = est_pose;
    this->prev_pose_array = pa;

    // Resample the particles
    this->resampleParticles();

    updating = false;
    return result;
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
        this->laserScanUpdating = true;
        // // Copy laser scan to latest laser scan field by field
        latestLaserScan.angle_min = laserScan.angle_min;
        latestLaserScan.angle_max = laserScan.angle_max;
        latestLaserScan.angle_increment = laserScan.angle_increment;
        latestLaserScan.time_increment = laserScan.time_increment;
        latestLaserScan.scan_time = laserScan.scan_time;
        latestLaserScan.range_min = laserScan.range_min;
        latestLaserScan.range_max = laserScan.range_max;
        latestLaserScan.ranges.size = laserScan.ranges.size;
        for (size_t i = 0; i < laserScan.ranges.size; i++)
        {
            latestLaserScan.ranges.data[i] = laserScan.ranges.data[i];
        }
    }
    this->laserScanUpdating = false;
}

geometry_msgs__msg__Pose ParticleFilter::etimatePose(){

    // Get the estimated pose
    SimplePose estimatedPose = SimplePose();
    estimatedPose.x = 0;
    estimatedPose.y = 0;
    estimatedPose.theta = 0;

    double totalWeight = 0;
    // Iterate through the particles and get the average pose
    for(Particle particle : this->particles){
        totalWeight += particle.weight;
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
    for(Particle particle : this->particles){
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
            if(cumsum.at(j) > r){
                index = j;
                break;
            }
        }
        // Create copy of the particle at index index
        Particle newParticle = Particle();
        newParticle.pose.position.x = this->particles.at(index).pose.position.x;
        newParticle.pose.position.y = this->particles.at(index).pose.position.y;
        newParticle.pose.position.z = this->particles.at(index).pose.position.z;
        newParticle.pose.orientation.x = this->particles.at(index).pose.orientation.x;
        newParticle.pose.orientation.y = this->particles.at(index).pose.orientation.y;
        newParticle.pose.orientation.z = this->particles.at(index).pose.orientation.z;
        newParticle.pose.orientation.w = this->particles.at(index).pose.orientation.w;
        newParticle.weight = 1.0/NUM_OF_PARTICLES;
        // Add the copied particle to the new particles list
        newParticles.push_back(newParticle);

    }

    // Clear the current particles
    this->particles.clear();

    // Copy the new particles to the current particles
    for(Particle particle : newParticles){
        this->particles.push_back(particle);
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

bool ParticleFilter::isUpdating(){
    // Check if the particle filter is updating
    return updating;
}