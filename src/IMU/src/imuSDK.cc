#include "imuSDK/imuSDK.h"
#include <iostream>
#include <cmath>

#define EARTH_RADIUS 6371000

// Constructor
ImuSDK::ImuSDK(std::string imu_path, boost::function<void(boost::shared_ptr<PIMUData>, double)> imu_callback)
    : imu_path(imu_path), imu_callback(imu_callback), running(false) {}

// Destructor
ImuSDK::~ImuSDK() {
    Stop();
}

// Start IMU Reading Thread
void ImuSDK::Start() {
    if (!running) {
        running = true;
        imu_thread = std::thread(&ImuSDK::ReadIMUData, this);
    }
}

// Stop IMU Reading Thread
void ImuSDK::Stop() {
    if (running) {
        running = false;
        if (imu_thread.joinable()) {
            imu_thread.join();
        }
    }
}

// Convert latitude/longitude to meters (approximate)
void latLonToMeters(double lat, double lon, double& x, double& y) {
    static const double refLat = 0.0;  // Set reference latitude (adjust as needed)
    static const double refLon = 0.0;  // Set reference longitude

    double dLat = (lat - refLat) * M_PI / 180.0;
    double dLon = (lon - refLon) * M_PI / 180.0;

    x = EARTH_RADIUS * dLon * cos(lat * M_PI / 180.0); // X-axis (meters)
    y = EARTH_RADIUS * dLat; // Y-axis (meters)
}

// Convert roll, pitch, yaw to quaternion
void rpyToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

double safeStod(const std::string& str) {
    try {
        return std::stod(str); // Try to convert the string to double
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: Invalid value '" << str << "' encountered during conversion." << std::endl;
        return NAN; // Return NaN to indicate invalid data
    }
}

// Read IMU Data from CSV and call the callback
void ImuSDK::ReadIMUData() {
    std::ifstream file(imu_path);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open IMU file " << imu_path << std::endl;
        return;
    }

    std::string line;
    std::getline(file, line);  // Skip header

    while (running && std::getline(file, line)) {
        std::stringstream ss(line);
        PIMUData imu_data;
        std::string value;
        double timestamp;
        double lat, lon, x_meters, y_meters;

        // Read values from CSV (indexing based on your format)
        std::getline(ss, value, ','); timestamp = safeStod(value);  // time
        std::getline(ss, value, ','); /* Ignore system_time */
        std::getline(ss, value, ','); lat = safeStod(value);  // latitude
        std::getline(ss, value, ','); lon = safeStod(value);  // longitude
        std::getline(ss, value, ','); imu_data.altitude = safeStod(value);  // altitude
        std::getline(ss, value, ','); /* Ignore azimuth */
        std::getline(ss, value, ','); /* Ignore mode */
        std::getline(ss, value, ','); /* Ignore satellites */
        std::getline(ss, value, ','); /* Ignore Time */
        std::getline(ss, value, ','); imu_data.accel_x = safeStod(value);
        std::getline(ss, value, ','); imu_data.accel_y = safeStod(value);
        std::getline(ss, value, ','); imu_data.accel_z = safeStod(value);
        std::getline(ss, value, ','); imu_data.gyro_x = safeStod(value);
        std::getline(ss, value, ','); imu_data.gyro_y = safeStod(value);
        std::getline(ss, value, ','); imu_data.gyro_z = safeStod(value);
        std::getline(ss, value, ','); imu_data.roll = safeStod(value);
        std::getline(ss, value, ','); imu_data.pitch = safeStod(value);
        std::getline(ss, value, ','); /* Ignore original_yaw */
        std::getline(ss, value, ','); imu_data.yaw = safeStod(value);

        // Check if any of the values are NaN (invalid)
        if (std::isnan(timestamp) || std::isnan(lat) || std::isnan(lon) || std::isnan(imu_data.altitude) ||
            std::isnan(imu_data.accel_x) || std::isnan(imu_data.accel_y) || std::isnan(imu_data.accel_z) ||
            std::isnan(imu_data.gyro_x) || std::isnan(imu_data.gyro_y) || std::isnan(imu_data.gyro_z) ||
            std::isnan(imu_data.roll) || std::isnan(imu_data.pitch) || std::isnan(imu_data.yaw)) {
            std::cerr << "Warning: Invalid data encountered in row, skipping." << std::endl;
            continue;  // Skip this row if any value is invalid
        }


        // Convert lat/lon to meters
        latLonToMeters(lat, lon, x_meters, y_meters);
        imu_data.latitude = x_meters;
        imu_data.longitude = y_meters;

        // Convert roll, pitch, yaw to quaternion
        double qx, qy, qz, qw;
        rpyToQuaternion(imu_data.roll, imu_data.pitch, imu_data.yaw, qx, qy, qz, qw);

        // assigning the quaternion values
        imu_data.qx = qx;
        imu_data.qy = qy;
        imu_data.qz = qz;
        imu_data.qw = qw;


        // Create shared_ptr and call callback
        boost::shared_ptr<PIMUData> imu_ptr(new PIMUData(imu_data));
        imu_callback(imu_ptr, timestamp);

        // Simulate real-time delay
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    file.close();
}