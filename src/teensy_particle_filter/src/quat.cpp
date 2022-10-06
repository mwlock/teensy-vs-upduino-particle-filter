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

double Quat::yawFromPose(geometry_msgs__msg__Pose pose) {
    Quaternion q;
    q.x = pose.orientation.x;
    q.y = pose.orientation.y;
    q.z = pose.orientation.z;
    q.w = pose.orientation.w;
    EulerAngles angles = ToEulerAngles(q);
    return angles.yaw;
}

// Function that returns a pose from a given x, y, yaw, pitch, roll
geometry_msgs__msg__Pose Quat::poseFromXYZRPY(double x, double y, double z, double roll, double pitch, double yaw) {
    // Create position
    geometry_msgs__msg__Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Create orientation
    Quaternion q = EulerToQuaternion(roll, pitch, yaw);
    pose.orientation.x = q.x;
    pose.orientation.y = q.y;
    pose.orientation.z = q.z;
    pose.orientation.w = q.w;
    return pose;
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

Quaternion Quat::EulerToQuaternion(double roll, double pitch, double yaw) {
    Quaternion q;

    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);

    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;

    return q;
}

// Checks that two poses are equal
bool Quat::arePosesEqual(geometry_msgs__msg__Pose pose1, geometry_msgs__msg__Pose pose2) {
    if (pose1.position.x == pose2.position.x && pose1.position.y == pose2.position.y && pose1.position.z == pose2.position.z) {
        if (pose1.orientation.x == pose2.orientation.x && pose1.orientation.y == pose2.orientation.y && pose1.orientation.z == pose2.orientation.z && pose1.orientation.w == pose2.orientation.w) {
            return true;
        }
    }
    return false;
}