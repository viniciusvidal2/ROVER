#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#ifndef SCANPREPROCESSOR_h
#define SCANPREPROCESSOR_h

class ScanPreprocessor
{
public:
    ScanPreprocessor(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params,
                     std::unordered_map<std::string, std::string> &frames);

    ~ScanPreprocessor() = default;

    // Scan callback
    void scanCallback(const livox_ros_driver2::CustomMsgConstPtr &scan_msg);
    
    // Imu callback
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);

private:
    // Filtered scan and imu publishers
    ros::Publisher out_scan_pub_, out_imu_pub_;
    // Raw scan subscriber
    ros::Subscriber scan_sub_;
    // Raw IMU subscriber
    ros::Subscriber imu_sub_;
    // Frames
    std::string in_frame_, out_frame_;
    Eigen::Matrix4f lidar_T_body_;
    Eigen::Quaternionf lidar_q_body_;
    // Filter parameters
    Eigen::Vector3f negative_range_, positive_range_;
};
#endif // SCANPREPROCESSOR_h
