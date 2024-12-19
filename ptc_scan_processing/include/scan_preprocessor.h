#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>

#ifndef SCANPREPROCESSOR_h
#define SCANPREPROCESSOR_h

class ScanPreprocessor
{
public:
    ScanPreprocessor(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params,
                     std::unordered_map<std::string, std::string> &frames);

    ~ScanPreprocessor() = default;

    // Cloud callback
    void scanCallback(const livox_ros_driver2::CustomMsgConstPtr &scan_msg);

private:
    // Filtered scan publisher
    ros::Publisher out_scan_pub_;
    // Raw scan subscriber
    ros::Subscriber scan_sub_;
    // Frames
    std::string in_frame_, out_frame_;
    Eigen::Matrix4f lidar_T_body_;
    // Filter parameters
    Eigen::Vector3f negative_range_, positive_range_;
};
#endif // SCANPREPROCESSOR_h
