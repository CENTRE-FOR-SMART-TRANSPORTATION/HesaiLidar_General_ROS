#ifndef INCLUDE_IMU_DATA_H
#define INCLUDE_IMU_DATA_H

#include <vector>
#include <ros/time.h>

// IMU Data Structure
struct PIMUData {
    ros::Time timestamp;
    double latitude, longitude, altitude;   // Position Data
    double accel_x, accel_y, accel_z;       // Linear Acceleration (m/s²)
    double gyro_x, gyro_y, gyro_z;          // Angular Velocity (rad/s)
    double roll, pitch, yaw;                // Orientation (degrees)
};

// IMU Data Container (similar to PPointCloud but for IMU)
typedef std::vector<PIMUData> PIMUCloud;

#endif // IMU_DATA_H
