#ifndef QUAT_HPP
#define QUAT_HPP

#define _USE_MATH_DEFINES
#define PI 3.1415926535897932384626433832795

#include <cmath>

#include <geometry_msgs/msg/pose.h>

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

class Quat {

    private:

    public:
            
        static EulerAngles ToEulerAngles(Quaternion q);
        static double angleDiff(double a, double b);
        static Quaternion EulerToQuaternion(EulerAngles angles);
        static Quaternion EulerToQuaternion(double roll,double pitch,double yaw);
        static double yawFromPose(geometry_msgs__msg__Pose pose);
        static geometry_msgs__msg__Pose poseFromXYZRPY(double x, double y, double z, double roll, double pitch, double yaw);
        static bool arePosesEqual(geometry_msgs__msg__Pose pose1, geometry_msgs__msg__Pose pose2);

};

#endif