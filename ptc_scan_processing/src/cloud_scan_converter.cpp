#include "cloud_scan_converter.h"

CloudScanConverter::CloudScanConverter(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params)
{
    // Subscribe to the input point cloud
    cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/localization/obstacle_ptc_lidar_frame", 1, &CloudScanConverter::cloudCallback, this);
    // Output with converted laser scan
    out_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/obstacle_avoidance/obstacles_scan", 1);

    // Convertion from point cloud to scan
    angle_resolution_ = params["angle_resolution"] * M_PI / 180.0f; // [rad]

    // Set the filter parameters
    min_intensity_ = params["min_intensity"];
    frontal_fov_ = params["frontal_fov"] * M_PI / 180.0f; // [rad]
    max_xy_range_ = params["max_xy_range"];
    vehicle_height_ = params["vehicle_height"]; // negative to get to the floor [m]
}

// Cloud callback
void CloudScanConverter::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<PointIn>::Ptr cloud_in(new pcl::PointCloud<PointIn>());
    pcl::fromROSMsg(*cloud_msg, *cloud_in);
    if (cloud_in->empty())
    {
        return;
    }

    // Output laser scan
    sensor_msgs::LaserScan scan_out;
    scan_out.header.stamp = cloud_msg->header.stamp;
    scan_out.header.frame_id = "body";
    scan_out.scan_time = 0.1; // 10 Hz

    // Work on the input point cloud
    std::vector<float> ranges, intensities, angles;
    ranges.reserve(cloud_in->points.size());
    intensities.reserve(cloud_in->points.size());
    angles.reserve(cloud_in->points.size());
    for (const auto &p : cloud_in->points)
    {
        // Apply the relative pose transformation
        const Eigen::Vector3f pp(p.x, p.y, p.z);

        // Filter Z values that are out of possible rover collision
        if (pp.z() < -vehicle_height_ + 0.2f || pp.z() > 0.5)
        {
            continue;
        }
        // Calculate and filter by range
        const float p_range = pp.head<2>().norm();
        if (p_range < max_xy_range_)
        {
            continue;
        }
        // Filter by intensity
        if (p.intensity < min_intensity_)
        {
            continue;
        }

        // Calculate the point angle
        // Z up, X forward, Y left
        // 0 rad is forward (X), positive is counter-clockwise
        float angle = std::atan2(pp.y(), pp.x()); // [rad]

        // Filter by frontal FOV
        if (abs(angle) > frontal_fov_ / 2.0f)
        {
            continue;
        }

        // Keep the angle between 0 and 2*pi
        if (angle < 0.0f)
        {
            angle += 2.0f * M_PI;
        }

        // Fill the output laser scan candidates
        angles.emplace_back(angle);
        ranges.emplace_back(p_range);
        intensities.emplace_back(p.intensity);
    }

    // Prapare the output laser scan values according to angle resolution
    if (!ranges.empty())
    {
        filterRangeAndIntensityVectors(ranges, intensities, angles);
        if (ranges.size() > 3)
        {
            // Publish the output laser scan
            scan_out.time_increment = scan_out.scan_time / static_cast<float>(ranges.size() - 1);
            scan_out.angle_min = min_scan_angle_;
            scan_out.angle_max = max_scan_angle_;
            scan_out.angle_increment = angle_resolution_;
            scan_out.range_min = *std::min_element(ranges.begin(), ranges.end());
            scan_out.range_max = *std::max_element(ranges.begin(), ranges.end());
            scan_out.ranges = ranges;
            scan_out.intensities = intensities;
            out_scan_pub_.publish(scan_out);
        }
    }
}

void CloudScanConverter::filterRangeAndIntensityVectors(std::vector<float> &ranges, std::vector<float> &intensities, std::vector<float> &angles)
{
    // Sort the ranges and intensities based on the angles
    std::vector<std::pair<float, std::pair<float, float>>> angles_ranges_intensities;
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        angles_ranges_intensities.emplace_back(std::make_pair(angles[i], std::make_pair(ranges[i], intensities[i])));
    }
    std::sort(angles_ranges_intensities.begin(), angles_ranges_intensities.end(),
              [](const std::pair<float, std::pair<float, float>> &a, const std::pair<float, std::pair<float, float>> &b)
              { return a.first < b.first; });

    // Filter the ranges and intensities based on the angle resolution
    const std::size_t n_readings = static_cast<std::size_t>((max_scan_angle_ - min_scan_angle_) / angle_resolution_);
    std::vector<float> filtered_ranges(n_readings), filtered_intensities(n_readings);
    float current_angle = min_scan_angle_;
    for (std::size_t i = 0; i < angles_ranges_intensities.size(); ++i)
    {
        std::size_t angle_index = static_cast<std::size_t>(angles_ranges_intensities[i].first / angle_resolution_);
        if (angle_index < n_readings)
        {
            if (filtered_ranges[angle_index] == 0.0f || angles_ranges_intensities[i].second.first < filtered_ranges[angle_index])
            {
                filtered_ranges[angle_index] = angles_ranges_intensities[i].second.first;
                filtered_intensities[angle_index] = angles_ranges_intensities[i].second.second;
            }
        }
    }

    // Fix the zero readings with very high value
    for (std::size_t i = 0; i < n_readings; ++i)
    {
        if (filtered_ranges[i] == 0.0f)
        {
            filtered_ranges[i] = 1e6f;
        }
    }

    // Update the original vectors
    ranges = filtered_ranges;
    intensities = filtered_intensities;
}
