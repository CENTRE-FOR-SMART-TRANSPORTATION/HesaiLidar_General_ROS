#ifndef INCLUDE_IMUSDK_H
#define INCLUDE_IMUSDK_H

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <imuSDK/imu_types.h>
#include <sensor_msgs/Imu.h>


#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <atomic>
#include <ros/time.h>

class ImuSDK
{
public:
    ImuSDK(std::string imu_path, boost::function<void(boost::shared_ptr<PIMUData>, double)> imu_callback);
    ~ImuSDK();

    void Start();
    void Stop();

private:
    void ReadIMUData();  // Internal function for reading data

    std::string imu_path;
    boost::function<void(boost::shared_ptr<PIMUData>, double)> imu_callback;
    std::thread imu_thread;
    std::atomic<bool> running;
};

#endif // INCLUDE_PANDARGENERAL_H_
