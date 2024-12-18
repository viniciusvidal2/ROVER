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
    CloudScanConverter(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params, std::unordered_map<std::string, bool> &flags,
                std::unordered_map<std::string, std::string> &frames) ;

    ~CloudScanConverter() = default;

    // Cloud callback
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
    /// @brief Filter range and intensity vectors based on the angle resolution
    /// @param ranges Vector of ranges
    /// @param intensities Vector of intensities
    /// @param angles Vector of angles
    void filterRangeAndIntensityVectors(std::vector<float>& ranges, std::vector<float>& intensities, std::vector<float>& angles);

    /// @brief Filter rover points
    /// @param p Point to be filtered
    /// @return True if the point should be filtered out, false otherwise
    const bool filterRoverPoints(const Eigen::Vector4f& p);

    /// @brief Filter by range
    /// @param p Point to be filtered
    /// @return True if the point should be filtered out, false otherwise
    const bool filterRange(const Eigen::Vector4f& p);

    /// @brief Filter by intensity
    /// @param intensity Point intensity to assess
    /// @return True if the point should be filtered out, false otherwise
    const bool filterIntensity(const float intensity);

    // Filtered point cloud publisher
    ros::Publisher debug_cloud_pub_, out_scan_pub_;
    // Raw point cloud subscriber
    ros::Subscriber cloud_sub_;
    // Filter parameters
    pcl::PointXYZ negative_range_, positive_range_; // [m]
    float min_intensity_; // [units]
    float max_xy_range_; // [m]
    float angle_resolution_; // [rad]
    float frontal_fov_; // [rad]
    const float min_scan_angle_ = 0, max_scan_angle_ = 2 * M_PI; // [rad]
    // Filter flags
    bool apply_filter_, filter_range_, filter_intensity_, filter_rover_points_, publish_debug_cloud_;
    // Frames
    std::string in_frame_, out_frame_;
    float x_in_out_, y_in_out_, z_in_out_, roll_in_out_, pitch_in_out_, yaw_in_out_;
    Eigen::Matrix4f out_T_in_;

    // Debug publishers
    ros::Publisher debug_pub_intensity_filter_pct_, debug_pub_range_filter_pct_, debug_pub_rover_filter_pct_;
    ros::Publisher debug_pub_total_filter_pct_;
};
#endif // CLOUDFILTER_H
