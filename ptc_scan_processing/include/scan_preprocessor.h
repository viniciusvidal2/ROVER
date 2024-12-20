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

    // Cloud callback
    void scanCallback(const livox_ros_driver2::CustomMsgConstPtr &scan_msg, const sensor_msgs::Imu::ConstPtr &imu_msg);

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

    /// @brief The synchronization policy
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        livox_ros_driver2::CustomMsg,
        sensor_msgs::Imu>;

    /// @brief Subscribers and synchronizer
    message_filters::Subscriber<livox_ros_driver2::CustomMsg> scan_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};
#endif // SCANPREPROCESSOR_h
