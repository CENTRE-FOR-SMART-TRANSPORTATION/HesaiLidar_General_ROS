#include <ros/ros.h>
#include "rosbag/bag.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include "imuSDK/imuSDK.h"
#include <fstream>
#include <mutex>

std::vector<sensor_msgs::Imu> imu_buffer;  // Buffer to hold IMU messages
const size_t BUFFER_SIZE = 100;  // Threshold for the number of IMU messages to write at once
// #define PRINT_FLAG 

using namespace std;

class HesaiLidarClient
{
public:
  HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh)
  {
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("hesai/pandar", 10);
    packetPublisher = node.advertise<hesai_lidar::PandarScan>("pandar_packets",10);
    imuPublisher = node.advertise<sensor_msgs::Imu>("imu_raw", 10);

    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;  // Get local correction when getting from lidar failed
    string lidarType;
    string frameId;
    int pclDataType;
    string pcapFile;
    string dataType;
    string multicastIp;
    bool coordinateCorrectionFlag;
    string targetFrame;
    string fixedFrame;
    string imuFile;
    string recordFile;
    bool gpsFlag;

    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("lidar_type", lidarType);
    nh.getParam("frame_id", frameId);
    nh.getParam("pcldata_type", pclDataType);
    nh.getParam("publish_type", m_sPublishType);
    nh.getParam("timestamp_type", m_sTimestampType);
    nh.getParam("data_type", dataType);
    nh.getParam("multicast_ip", multicastIp);
    nh.getParam("coordinate_correction_flag", coordinateCorrectionFlag);
    nh.getParam("target_frame", targetFrame);
    nh.getParam("fixed_frame", fixedFrame);
    nh.getParam("record_file", recordFile);
    nh.getParam("imu_file", imuFile);
    nh.getParam("imu_file_has_gps",gpsFlag);
  
    if(!pcapFile.empty()){
      hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, lidarCorrectionFile, \
      coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        std::ifstream fin(lidarCorrectionFile);
        if (fin.is_open()) {
          std::cout << "Open correction file " << lidarCorrectionFile << " succeed" << std::endl;
          int length = 0;
          std::string strlidarCalibration;
          fin.seekg(0, std::ios::end);
          length = fin.tellg();
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[length];
          fin.read(buffer, length);
          fin.close();
          strlidarCalibration = buffer;
          int ret = hsdk->LoadLidarCorrectionFile(strlidarCalibration);
          if (ret != 0) {
            std::cout << "Load correction file from " << lidarCorrectionFile <<" failed" << std::endl;
          } else {
            std::cout << "Load correction file from " << lidarCorrectionFile << " succeed" << std::endl;
          }
        }
        else{
          std::cout << "Open correction file " << lidarCorrectionFile << " failed" << std::endl;
        }
      }
    }
    else if ("rosbag" == dataType){
      hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, \
      lidarCorrectionFile, coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        packetSubscriber = node.subscribe("pandar_packets",10,&HesaiLidarClient::scanCallback, (HesaiLidarClient*)this, ros::TransportHints().tcpNoDelay(true));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        boost::bind(&HesaiLidarClient::gpsCallback, this, _1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId,\
         m_sTimestampType, lidarCorrectionFile, multicastIp, coordinateCorrectionFlag, targetFrame, fixedFrame);
    }
    
    if (hsdk != NULL) {
        hsdk->Start();
        // hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
    } else {
        printf("create sdk fail\n");
    }

    if (!imuFile.empty()){
      imusdk = new ImuSDK(imuFile, boost::bind(&HesaiLidarClient::imuCallback, this, _1, _2),gpsFlag);
      if (imusdk != NULL){
        imusdk->Start();
      } else {
        printf("IMU failed\n");
      }
    }

    if (!recordFile.empty()){
      bag.open(recordFile, rosbag::bagmode::Write);
      if (bag.isOpen()) {
          ROS_INFO("Bag opened successfully: %s", recordFile.c_str());
          bag.setChunkThreshold(1024 * 1024 * 10);  // Set buffer size to 10 MB
      } else {
          ROS_ERROR("Failed to open bag: %s", recordFile.c_str());
      }
      
    }
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::PandarScanPtr scan)
  {
      if (m_sPublishType == "both" || m_sPublishType == "points") {
          pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
          sensor_msgs::PointCloud2 output;
          pcl::toROSMsg(*cld, output);
          lidarPublisher.publish(output);
  
          // Synchronize access to the bag
          std::lock_guard<std::mutex> lock(bag_mutex);
          if (bag.isOpen()) {
              try {
                  bag.write("/hesai/pandar", ros::Time(timestamp), output);
              } catch (rosbag::BagException& e) {
                  ROS_ERROR("Failed to write to bag: %s", e.what());
              }
          }
  #ifdef PRINT_FLAG
          printf("timestamp: %f, point size: %ld.\n", timestamp, cld->points.size());
  #endif        
      }
  
      if (m_sPublishType == "both" || m_sPublishType == "raw") {
          packetPublisher.publish(scan);
  #ifdef PRINT_FLAG
          printf("raw size: %d.\n", scan->packets.size());
  #endif
      }
  }

  void imuCallback(boost::shared_ptr<PIMUData> imu_data, double timestamp)
  {
      if (!imu_data) {
          ROS_WARN("Received null IMU data.");
          return;
      }
  
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = ros::Time(timestamp);
      imu_msg.header.frame_id = "imu_link";  // Update based on your TF setup
  
      // Fill IMU message fields
      imu_msg.linear_acceleration.x = imu_data->accel_x;
      imu_msg.linear_acceleration.y = imu_data->accel_y;
      imu_msg.linear_acceleration.z = imu_data->accel_z;
  
      imu_msg.angular_velocity.x = imu_data->gyro_x;
      imu_msg.angular_velocity.y = imu_data->gyro_y;
      imu_msg.angular_velocity.z = imu_data->gyro_z;
  
      imu_msg.orientation.w = imu_data->qw;
      imu_msg.orientation.x = imu_data->qx;
      imu_msg.orientation.y = imu_data->qy;
      imu_msg.orientation.z = imu_data->qz;

      imu_msg.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
      imu_msg.angular_velocity_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
      imu_msg.linear_acceleration_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

      imuPublisher.publish(imu_msg);
  
      // Synchronize access to the bag
      // std::lock_guard<std::mutex> lock(bag_mutex);
      // if (bag.isOpen()) {
      //     try {
      //         bag.write("/imu", ros::Time(timestamp), imu_msg);
      //     } catch (rosbag::BagException& e) {
      //         ROS_ERROR("Failed to write to bag: %s", e.what());
      //     }
      // }

      // Add IMU message to the buffer
    imu_buffer.push_back(imu_msg);

    // Check if the buffer size has reached the threshold
    if (imu_buffer.size() >= BUFFER_SIZE) {
        // Lock to ensure thread-safe access to the rosbag
        printf("Buffer size %d \n", imu_buffer.size());

        std::lock_guard<std::mutex> lock(bag_mutex);
        if (bag.isOpen()){
        // Write all buffered IMU messages to the bag
          for (const auto& imu_iterate : imu_buffer) {
              try {
                  bag.write("/imu", imu_iterate.header.stamp, imu_iterate);
              } catch (const rosbag::BagException& e) {
                  ROS_ERROR("Failed to write IMU data to bag: %s", e.what());
              }
          }
        }
        // Clear the buffer after writing
        imu_buffer.clear();
    }

  
  #ifdef PRINT_FLAG
      printf("IMU Timestamp: %f | Accel: (%.3f, %.3f, %.3f) | Gyro: (%.3f, %.3f, %.3f)\n",
             timestamp,
             imu_data->accel_x, imu_data->accel_y, imu_data->accel_z,
             imu_data->gyro_x, imu_data->gyro_y, imu_data->gyro_z);
  #endif    
  }

  void gpsCallback(int timestamp) {
#ifdef PRINT_FLAG
      printf("gps: %d\n", timestamp);
#endif      
  }

  void scanCallback(const hesai_lidar::PandarScanPtr scan)
  {
    // printf("pandar_packets topic message received,\n");
    hsdk->PushScanPacket(scan);
  }

  ~HesaiLidarClient()
  {
      std::lock_guard<std::mutex> lock(bag_mutex);  // Synchronize access
      if (bag.isOpen()) {
          bag.close();
          ROS_INFO("Bag closed successfully.");
      }
  }


private:
  ros::Publisher lidarPublisher;
  ros::Publisher packetPublisher;
  ros::Publisher imuPublisher;
  PandarGeneralSDK* hsdk;
  ImuSDK* imusdk;
  string m_sPublishType;
  string m_sTimestampType;
  ros::Subscriber packetSubscriber;
  rosbag::Bag bag;
  std::mutex bag_mutex;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandar");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiLidarClient pandarClient(node, nh);

  ros::spin();
  return 0;
}
