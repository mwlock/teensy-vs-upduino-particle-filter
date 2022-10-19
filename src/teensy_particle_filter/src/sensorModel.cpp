#include "sensorModel.hpp"

SensorModel::SensorModel() {}

SensorModel::SensorModel(std::vector<std::tuple<double, double>> map_obstacles)
{
    this->map_obstacles = map_obstacles;
}

double SensorModel::sampleSensorModel(
    geometry_msgs__msg__Pose particlePose, 
    sensor_msgs__msg__LaserScan laserScan, 
    void (*printDebug)(const char*))
    {

    // Check if the laser scan is empty
    if (laserScan.ranges.size == 0){
        return 1.0;
    }

    // Set the particle probability to 1
    double particleProbability = 1;

    // Determine map coordinates of the particle
    SimplePose mapPose = calculateMapPose(particlePose);

    // Angle of the laser scan
    double angle = laserScan.angle_min;

    
    // ==================================================================================
    // Loop through all the laser scan ranges
    // ==================================================================================
    for(int i = 0; i < (uint16_t) laserScan.ranges.size; i++){

        // Get the range of the laser scan
        double range = laserScan.ranges.data[i];

        // Check if the range is valid
        if(!(range < laserScan.range_min || range > laserScan.range_max)){   


            // Project the range onto the map
            double x = mapPose.x + range * cos(mapPose.theta + angle);
            double y = mapPose.y + range * sin(mapPose.theta + angle);

            // Find the closest obstacle to the particle
            double obstacleDistance = this->closestObstacle(x, y, printDebug);

            // Calculate the probability of the particle
            double probailityOfHit = calculateProbability(obstacleDistance,printDebug);
            
            // Multiply the particle probability by the probability of the laser scan
            particleProbability *= probailityOfHit;
        }

        // Increment the angle of the laser scan
        angle += laserScan.angle_increment;

    }
    // ==================================================================================

    return particleProbability;
}

SimplePose SensorModel::calculateMapPose(geometry_msgs__msg__Pose particlePose){

    // Create a simple pose
    SimplePose mapPose;

    // Calculate the map pose
    mapPose.x = particlePose.position.x;
    mapPose.y = particlePose.position.y;
    mapPose.theta = Quat::yawFromPose(particlePose);

    // Determine the pose of the particle in the map
    mapPose.x = mapPose.x - MAP_ORIGIN_X;
    mapPose.y = mapPose.y - MAP_ORIGIN_Y;   

    return mapPose;
}

void SensorModel::calculateGridPose(float x_input, float y_input, int32_t* xy_output){

    // Add the map origin to the x and y
    float x = x_input;
    float y = y_input;

    int MAP_HEIGHT =  sizeof(map_array) / sizeof(map_array[0]);
    int MAP_WIDTH = sizeof(map_array[0]) / sizeof(bool);

    // Calculate the grid pose
    int16_t x_grid = (int16_t) round(x / MAP_RESOLUTION);
    int16_t y_grid = MAP_HEIGHT - ((int16_t) round(y / MAP_RESOLUTION));
    *xy_output =  (((int32_t)x_grid) << 16) | y_grid;
}

double SensorModel::closestObstacle(double x, double y, void (*printDebug)(const char*)){

    // Find the closest obstacle to the particle in map_particle vector of tuple
    double closestObstacle = std::numeric_limits<double>::max();
    double closestObstacleIndex_x = 0;
    double closestObstacleIndex_y = 0;
    for (auto obstacle : this->map_obstacles){
        double obstacleX = std::get<0>(obstacle);
        double obstacleY = std::get<1>(obstacle);
        double distance = sqrt(pow(x - obstacleX, 2) + pow(y - obstacleY, 2));
        if (distance < closestObstacle){
            closestObstacle = distance;
            closestObstacleIndex_x = obstacleX;
            closestObstacleIndex_y = obstacleY;
        }
    }

    return closestObstacle;
}

double SensorModel::calculateProbability(double range,void (*printDebug)(const char*)){

    // Evaluate guassian with std of "LIKELIHOOD_STD_DEV" at "range"
    double probability = 1 / (LIKELIHOOD_STD_DEV * sqrt(2 * PI)) * exp(-pow(range, 2) / (2 * pow(LIKELIHOOD_STD_DEV, 2)));

    return probability;
}