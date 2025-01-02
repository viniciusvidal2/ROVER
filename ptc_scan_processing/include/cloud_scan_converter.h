#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>

#ifndef CLOUDFILTER_H
#define CLOUDFILTER_H

// Type defs for point types PCL
typedef pcl::PointXYZI PointIn;


class CloudScanConverter 
{
public:
    CloudScanConverter(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params);

    ~CloudScanConverter() = default;

    // Cloud callback
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
    /// @brief Filter range and intensity vectors based on the angle resolution
    /// @param ranges Vector of ranges
    /// @param intensities Vector of intensities
    /// @param angles Vector of angles
    void filterRangeAndIntensityVectors(std::vector<float>& ranges, std::vector<float>& intensities, std::vector<float>& angles);

    // Filtered point cloud publisher
    ros::Publisher out_scan_pub_;
    // Raw point cloud subscriber
    ros::Subscriber cloud_sub_;
    // Filter parameters
    float min_intensity_; // [units]
    float max_xy_range_; // [m]
    float vehicle_height_; // [m]
    float angle_resolution_; // [rad]
    float frontal_fov_; // [rad]
    const float min_scan_angle_ = 0, max_scan_angle_ = 2 * M_PI; // [rad]
};
#endif // CLOUDFILTER_H
