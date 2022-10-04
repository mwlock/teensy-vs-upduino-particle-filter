#ifndef QUAT_HPP
#define QUAT_HPP

#define _USE_MATH_DEFINES
#define PI 3.1415926535897932384626433832795

#include <cmath>

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

};

// double angleDiff(double angle1, double angle2){
//     double d1 = angle1 - angle2;
//     double d2 = 2 * PI - abs(d1);
    
//     if (d1 > 0.0) 
//         d2 *= -1.0;

//     if (abs(d1) < abs(d2))
//         return d1;

//     else 
//         return d2;
// }


// EulerAngles ToEulerAngles(Quaternion q) {
//     EulerAngles angles;

//     // roll (x-axis rotation)
//     double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
//     double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
//     angles.roll = std::atan2(sinr_cosp, cosr_cosp);

//     // pitch (y-axis rotation)
//     double sinp = 2 * (q.w * q.y - q.z * q.x);
//     if (std::abs(sinp) >= 1)
//         angles.pitch = std::copysign(PI / 2, sinp); // use 90 degrees if out of range
//     else
//         angles.pitch = std::asin(sinp);

//     // yaw (z-axis rotation)
//     double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
//     double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
//     angles.yaw = std::atan2(siny_cosp, cosy_cosp);

//     return angles;
// }

// Quaternion EulerToQuaternion(EulerAngles angles) {
//     Quaternion q;

//     double cy = std::cos(angles.yaw * 0.5);
//     double sy = std::sin(angles.yaw * 0.5);
//     double cr = std::cos(angles.roll * 0.5);
//     double sr = std::sin(angles.roll * 0.5);
//     double cp = std::cos(angles.pitch * 0.5);
//     double sp = std::sin(angles.pitch * 0.5);

//     q.w = cy * cr * cp + sy * sr * sp;
//     q.x = cy * sr * cp - sy * cr * sp;
//     q.y = cy * cr * sp + sy * sr * cp;
//     q.z = sy * cr * cp - cy * sr * sp;

//     return q;
// }

#endif