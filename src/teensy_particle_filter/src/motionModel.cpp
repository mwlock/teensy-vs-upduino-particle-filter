#include  "motionModel.hpp"

MotionModel::MotionModel() {
}

geometry_msgs__msg__Pose MotionModel::sampleMotionModel(
        geometry_msgs__msg__Pose previous_xt, // previous pose at time t
        geometry_msgs__msg__Pose latestOdom,
        geometry_msgs__msg__Pose prevOdom
    ){
        // TODO : check previous odom == last odom and return previousPose??

        // Calculate pose delta (uses latestOdom and prevOdom)
        // declare variables
        double x,y,theta;
        double d1, dt,d2;
        {   
            // Get x,y, theta from previous particle position
            x = prevOdom.position.x;
            y = prevOdom.position.y;            
            double xq,yq,zq,wq;
            xq = prevOdom.orientation.x;
            yq = prevOdom.orientation.y;
            zq = prevOdom.orientation.z;
            wq = prevOdom.orientation.w;
            Quaternion quat = {wq,xq,yq,zq};
            EulerAngles eulerAngles = Quat::ToEulerAngles(quat); 
            theta = eulerAngles.yaw;

            double x_prime = previous_xt.position.x;
            double y_prime = previous_xt.position.y;
            xq = previous_xt.orientation.x;
            yq = previous_xt.orientation.y;
            zq = previous_xt.orientation.z;
            wq = previous_xt.orientation.w;
            quat = {wq,xq,yq,zq};
            eulerAngles = Quat::ToEulerAngles(quat); 
            double theta_prime = eulerAngles.yaw;

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
        }

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
        noised1 = Distributions::sampleNormal(0, std_dev_d1);
        noisedt = Distributions::sampleNormal(0, std_dev_dt);
        noised2 = Distributions::sampleNormal(0, std_dev_d2);

        double t_d1 = Quat::angleDiff(d1,noised1);
        double t_dt = dt + noisedt;
        double t_d2 = Quat::angleDiff(d2,noised2);
        
        // Calculate new pose
        double new_x = x + (t_dt * cos(theta + t_d1));
        double new_y = y + (t_dt * sin(theta + t_d1));
        double new_theta = theta + t_d1 + t_d2;

        // Convert to quaternion
        EulerAngles eulerAngles = {0, 0, new_theta};
        Quaternion quat = Quat::EulerToQuaternion(eulerAngles);
        double new_xq = quat.x;
        double new_yq = quat.y;
        double new_zq = quat.z;
        double new_wq = quat.w;

        // Create new pose
        geometry_msgs__msg__Pose newPose;
        newPose.position.x = new_x;
        newPose.position.y = new_y;
        newPose.position.z = 0.0;
        newPose.orientation.x = new_xq;
        newPose.orientation.y = new_yq;
        newPose.orientation.z = new_zq;
        newPose.orientation.w = new_wq;

        return newPose;        

    }   
