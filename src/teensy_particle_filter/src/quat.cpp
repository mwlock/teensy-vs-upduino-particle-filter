#include "quat.hpp"

EulerAngles Quat::ToEulerAngles(Quaternion q) {
        EulerAngles angles;

        // roll (x-axis rotation)
        double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        angles.roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = +2.0 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            angles.pitch = std::copysign(PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
        angles.yaw = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }

double Quat::angleDiff(double a, double b) {
    double diff = a - b;
    while (diff > PI) diff -= 2*PI;
    while (diff < -PI) diff += 2*PI;
    return diff;
}

Quaternion Quat::EulerToQuaternion(EulerAngles angles) {
    Quaternion q;

    double cy = std::cos(angles.yaw * 0.5);
    double sy = std::sin(angles.yaw * 0.5);
    double cr = std::cos(angles.roll * 0.5);
    double sr = std::sin(angles.roll * 0.5);
    double cp = std::cos(angles.pitch * 0.5);
    double sp = std::sin(angles.pitch * 0.5);

    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;

    return q;
}