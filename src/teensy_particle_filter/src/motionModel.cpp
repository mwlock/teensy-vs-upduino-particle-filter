#include  "motionModel.hpp"

MotionModel::MotionModel() {
}

geometry_msgs__msg__Pose MotionModel::sampleMotionModel(
        geometry_msgs__msg__Pose previous_xt, // previous pose at time t
        geometry_msgs__msg__Pose latestOdom,
        geometry_msgs__msg__Pose prevOdom
    ){
        //check previous odom == last odom and return previousPose
        if (Quat::arePosesEqual(latestOdom, prevOdom)) {
            return previous_xt;
        }
        
        // Calculate pose delta (uses latestOdom and prevOdom)
        double d1, dt, d2;
        
        std::tie(d1, dt, d2) = MotionModel::getPoseDelta(latestOdom, prevOdom);

        // Uncertainties
        double alpha1 = ALPHA1;
        double alpha2 = ALPHA2;
        double alpha3 = ALPHA4;
        double alpha4 = ALPHA4;

        // Standard deviation
        double std_dev_d1 = sqrt((alpha1 * pow(d1,2)) + (alpha2 * pow(dt,2)));
        double std_dev_dt = sqrt((alpha3 * pow(dt,2)) + (alpha4 * pow(d1,2)) + (alpha4 * pow(d2,2)));
        double std_dev_d2 = sqrt((alpha1 * pow(d2,2)) + (alpha2 * pow(dt,2)));

        // Determine noise on motion model
        double noised1 = 0.0;
        double noisedt = 0.0;
        double noised2 = 0.0;
        
        // Sample from normal distribution
        // noised1 = normalDistribution(0, std_dev_d1);
        // noisedt = normalDistribution(0, std_dev_dt);
        // noised2 = normalDistribution(0, std_dev_d2);

        double t_d1 = Quat::angleDiff(d1,noised1);
        double t_dt = dt + noisedt;
        double t_d2 = Quat::angleDiff(d2,noised2);
        
        // Calculate new pose
        double curr_x = previous_xt.position.x;
        double curr_y = previous_xt.position.y;
        double curr_yaw = Quat::yawFromPose(previous_xt);

        double x = curr_x + (t_dt * cos(curr_yaw + t_d1));
        double y = curr_y + (t_dt * sin(curr_yaw + t_d1));
        double yaw = curr_yaw + t_d1 + t_d2;

        // Get new pose
        geometry_msgs__msg__Pose newPose = Quat::poseFromXYZRPY(x,y,0,0,0,yaw);
        return newPose;        
    }   
    
    
// getPoseDelta
// Calculates the change in pose between two poses

std::tuple<double, double, double> MotionModel::getPoseDelta(
        geometry_msgs__msg__Pose xt, // previous pose at time t
        geometry_msgs__msg__Pose previous_xt
    ){
        // Calculate pose delta (uses latestOdom and prevOdom)
        double d1, dt, d2;
        
        // Get x,y, theta from previous particle position
        // ----------------------------------------------
        double x, y, theta;
        x = xt.position.x;
        y = xt.position.y;            
        theta = Quat::yawFromPose(xt);
        // ----------------------------------------------

        // Get x_prime, y_prime, theta_prime from previous particle position
        // ----------------------------------------------
        double x_prime = previous_xt.position.x;
        double y_prime = previous_xt.position.y;
        double theta_prime = Quat::yawFromPose(previous_xt);
        // ----------------------------------------------

        // Calculate deltas
        double delta_translation = sqrt( pow(x-x_prime,2) + pow(y-y_prime,2) );
        double delta_rotation1 = 0.0;
        if (delta_translation > MOVED_TOO_CLOSE){
            delta_rotation1 = Quat::angleDiff(atan2(y_prime - y, x_prime - x), theta);
        }
        double delta = Quat::angleDiff(theta_prime, theta);
        double delta_rotation2 = Quat::angleDiff(delta, delta_rotation1);

        // return delta_rotation1, delta_translation, delta_rotation2
        d1 = delta_rotation1;
        dt = delta_translation;
        d2 = delta_rotation2;

        return std::make_tuple(d1, dt, d2);
    }   
