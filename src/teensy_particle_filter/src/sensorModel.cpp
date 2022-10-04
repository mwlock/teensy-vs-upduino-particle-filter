#include "sensorModel.hpp"

SensorModel::SensorModel()
{
}

double SensorModel::sampleSensorModel(geometry_msgs__msg__Pose particlePose, sensor_msgs__msg__LaserScan laserScan){

    // Check if the laser scan is empty
    if (laserScan.ranges.size == 0){
        return 0;
    }

    // Set the particle probability to 1
    double particleProbability = 1;

    // Determine map coordinates of the particle
    SimplePose mapPose = calculateMapPose(particlePose);

    // Angle of the laser scan
    double angle = laserScan.angle_min;

    // Loop through all the laser scan ranges
    for(int i = 0; i < (uint16_t) laserScan.ranges.size; i++){

        // Get the range of the laser scan
        double range = laserScan.ranges.data[i];

        // Check if the range is valid
        if(!(range < laserScan.range_min || range > laserScan.range_max)){            

            // Project the range onto the map
            double x = mapPose.x + range * cos(mapPose.theta + angle);
            double y = mapPose.y + range * sin(mapPose.theta + angle);

            // Calculate dimensions of the map
            int MAP_HEIGHT =  sizeof(map_array) / sizeof(map_array[0]); 
            int MAP_WIDTH = sizeof(map_array[0]) / sizeof(bool); 

            // Check if the laser scan is within the map and bound x and y       
            if (x > MAP_WIDTH * MAP_RESOLUTION) x = (MAP_WIDTH-1) * MAP_RESOLUTION;        
            if (y > MAP_HEIGHT * MAP_RESOLUTION) y = (MAP_HEIGHT-1) * MAP_RESOLUTION;

            // Find the closest obstacle to the particle
            double obstacleDistance = closestObstacle(x, y);

            // Calculate the probability of the particle
            double probailityOfHit = calculateProbability(obstacleDistance);
            
            // Multiply the particle probability by the probability of the laser scan
            particleProbability *= probailityOfHit;

        }

        // Increment the angle of the laser scan
        angle += laserScan.angle_increment;

    }
    return particleProbability;
}

SimplePose SensorModel::calculateMapPose(geometry_msgs__msg__Pose particlePose){

    // Calculate the map pose of the particle

    // Create a simple pose
    SimplePose mapPose;

    // Calculate the map pose
    mapPose.x = particlePose.position.x;
    mapPose.y = particlePose.position.y;
    Quaternion q = {particlePose.orientation.w, particlePose.orientation.x, particlePose.orientation.y, particlePose.orientation.z};
    mapPose.theta = Quat::ToEulerAngles(q).yaw;

    // Determine the pose of the particle in the map
    mapPose.x = mapPose.x - MAP_ORIGIN_X;
    mapPose.y = mapPose.y - MAP_ORIGIN_Y;   

    return mapPose;
}

double SensorModel::closestObstacle(double x, double y){

    // Calculate dimensions of the map
    int MAP_HEIGHT =  sizeof(map_array) / sizeof(map_array[0]); 
    int MAP_WIDTH = sizeof(map_array[0]) / sizeof(bool); 

    // Find the closest obstacle to the particle
    double closestObstacle = 1000000;
    for(int i = 0; i < MAP_HEIGHT; i++){
        for(int j = 0; j < MAP_WIDTH; j++){
            if(map_array[i][j]){
                double distance = sqrt(pow(x - j * MAP_RESOLUTION, 2) + pow(y - i * MAP_RESOLUTION, 2));
                if(distance < closestObstacle){
                    closestObstacle = distance;
                }
            }
        }
    }

    return closestObstacle;
}

double SensorModel::calculateProbability(double range){

    // Evaluate guassian with std of "LIKELIHOOD_STD_DEV" at "range"
    double probability = 1 / (LIKELIHOOD_STD_DEV * sqrt(2 * PI)) * exp(-pow(range, 2) / (2 * pow(LIKELIHOOD_STD_DEV, 2)));

    // Bound the probability
    if(probability < 0){
        probability = 0;
    } else if(probability > 1){
        probability = 1;
    }

    return probability;
}
