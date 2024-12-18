#include "cloud_filter.h"


CloudFilter::CloudFilter(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params, std::unordered_map<std::string, bool> &flags,
                        std::unordered_map<std::string, std::string> &frames) 
{
    apply_filter_ = flags["apply_filter"];
    publish_debug_cloud_ = flags["debug_cloud"];

    // Subscribe to the input point cloud
    cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/localization/obstacle_ptc_lidar_frame", 1, &CloudFilter::cloudCallback, this);
    // Advertise the filtered point cloud
    if (publish_debug_cloud_)
    {
        debug_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_avoidance/lidar_cloud_debug", 1);
    }
    out_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/obstacle_avoidance/obstacles_scan", 1);

    // Convertion from point cloud to scan
    angle_resolution_ = params["angle_resolution"]*M_PI/180.0f; // [rad]

    // Input and output frames variables
    in_frame_ = frames["in_frame"];
    out_frame_ = frames["out_frame"];
    x_in_out_ = params["x_in_out"];
    y_in_out_ = params["y_in_out"];
    z_in_out_ = params["z_in_out"];
    roll_in_out_ = params["roll_in_out"] * M_PI / 180.0f; // [rad]
    pitch_in_out_ = params["pitch_in_out"] * M_PI / 180.0f; // [rad]
    yaw_in_out_ = params["yaw_in_out"] * M_PI / 180.0f; // [rad]
    // Create homogeneous matrix from the relative pose
    Eigen::Matrix3f out_R_in = (Eigen::AngleAxisf(roll_in_out_, Eigen::Vector3f::UnitX())
                        * Eigen::AngleAxisf(pitch_in_out_, Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(yaw_in_out_, Eigen::Vector3f::UnitZ())).toRotationMatrix();
    out_T_in_ << out_R_in(0, 0), out_R_in(0, 1), out_R_in(0, 2), x_in_out_,
                out_R_in(1, 0), out_R_in(1, 1), out_R_in(1, 2), y_in_out_,
                out_R_in(2, 0), out_R_in(2, 1), out_R_in(2, 2), z_in_out_,
                0.0, 0.0, 0.0, 1.0;

    if (apply_filter_)
    {
        // Set the filter parameters
        min_intensity_ = params["min_intensity"];
        frontal_fov_ = params["frontal_fov"] * M_PI / 180.0f; // [rad]
        max_xy_range_ = params["max_xy_range"];
        const float rover_lengh = params["rover_lengh"];
        const float rover_width = params["rover_width"];
        const float livox_height_from_floor = params["livox_height_from_floor"];
        // The offsets are in the rover body frame
        // X forward, Y left, Z up
        negative_range_.x = -rover_lengh / 2.0;
        negative_range_.y = -rover_width / 2.0;
        negative_range_.z = livox_height_from_floor;
        positive_range_.x = rover_lengh / 2.0;
        positive_range_.y = rover_width / 2.0;
        positive_range_.z = 0.5;

        // Set the filter flags
        filter_range_ = flags["filter_range"];
        filter_rover_points_ = flags["filter_rover_points"];
        filter_intensity_ = flags["filter_intensity"];
    }

    // Debug publishers
    debug_pub_intensity_filter_pct_ = nh.advertise<std_msgs::Float32>("/debug/intensity_filter_pct", 1);
    debug_pub_range_filter_pct_ = nh.advertise<std_msgs::Float32>("/debug/range_filter_pct", 1);
    debug_pub_rover_filter_pct_ = nh.advertise<std_msgs::Float32>("/debug/rover_filter_pct", 1);
    debug_pub_total_filter_pct_ = nh.advertise<std_msgs::Float32>("/debug/total_filter_pct", 1);
}

// Cloud callback
void CloudFilter::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<PointIn>::Ptr cloud_in (new pcl::PointCloud<PointIn>());
    pcl::fromROSMsg(*cloud_msg, *cloud_in);
    if (cloud_in->empty()) 
    {
        return;
    }
    
    // Debug filter percentage values
    float intensity_filter_pct = 0.0f, range_filter_pct = 0.0f, rover_filter_pct = 0.0f;
    
    // Output debug point cloud
    pcl::PointCloud<PointIn>::Ptr cloud_out (new pcl::PointCloud<PointIn>());
    cloud_out->points.reserve(cloud_in->points.size());
    cloud_out->header.frame_id = out_frame_;
    // Output laser scan
    sensor_msgs::LaserScan scan_out;
    scan_out.header.stamp = cloud_msg->header.stamp;
    scan_out.header.frame_id = out_frame_;
    scan_out.scan_time = 0.1; // 10 Hz

    // Work on the input point cloud
    std::vector<float> ranges, intensities, angles;
    ranges.reserve(cloud_in->points.size());
    intensities.reserve(cloud_in->points.size());
    angles.reserve(cloud_in->points.size());
    for (const auto& p : cloud_in->points) 
    {
        // Apply the relative pose transformation
        const Eigen::Vector4f p_in(p.x, p.y, p.z, 1.0);
        const Eigen::Vector4f p_out(out_T_in_ * p_in);

        // Filter Z values that are out of possible rover collision
        if (p_out.z() < negative_range_.z || p_out.z() > positive_range_.z) 
        {
            continue;
        }

        // Filter rover points
        if (apply_filter_ && filter_rover_points_) 
        {
            if (filterRoverPoints(p_out)) 
            {
                ++rover_filter_pct;
                continue;
            }
        }

        // Filter by range
        if (apply_filter_ && filter_range_) 
        {
            if (filterRange(p_out)) 
            {
                ++range_filter_pct;
                continue;
            }
        }

        // Filter by intensity
        if (apply_filter_ && filter_intensity_) 
        {
            if (filterIntensity(p.intensity)) 
            {
                ++intensity_filter_pct;
                continue;
            }
        }
        
        // Calculate the angle and range of the point
        // Z positive, X forward, Y left
        // 0 rad is forward (X), positive is counter-clockwise
        const float range = std::sqrt(p_out.x()*p_out.x() + p_out.y()*p_out.y());
        float angle = std::atan2(p_out.y(), p_out.x()); // [rad]

        // Filter by frontal FOV
        if (abs(angle) > frontal_fov_/2.0f)
        {
            continue;
        }

        // Keep the angle between 0 and 2*pi
        if (angle < 0.0f) 
        {
            angle += 2.0f*M_PI;
        }        
        
        // Fill the output laser scan candidates
        angles.emplace_back(angle);
        ranges.emplace_back(range);
        intensities.emplace_back(p.intensity);

        // Add the point to the debug cloud
        if (publish_debug_cloud_) 
        {
            PointIn p_out_pcl;
            p_out_pcl.x = p_out.x();
            p_out_pcl.y = p_out.y();
            p_out_pcl.z = p_out.z();
            p_out_pcl.intensity = p.intensity;
            cloud_out->points.emplace_back(p_out_pcl);
        }
    }

    // Publish the debug cloud
    if (!cloud_out->points.empty() && publish_debug_cloud_) 
    {
        sensor_msgs::PointCloud2 out_msg;
        out_msg.header = cloud_msg->header;
        pcl::toROSMsg(*cloud_out, out_msg);
        debug_cloud_pub_.publish(out_msg);
    }

    // Prapare the output laser scan values according to angle resolution
    if (!ranges.empty()) 
    {
        filterRangeAndIntensityVectors(ranges, intensities, angles);
        if (ranges.size() > 3)
        {
            // Publish the output laser scan
            scan_out.time_increment = scan_out.scan_time/static_cast<float>(ranges.size() - 1);
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

    // Publish the debug filter percentage values
    std_msgs::Float32 pct_msg;
    pct_msg.data = 100.0f*intensity_filter_pct/static_cast<float>(cloud_in->points.size());
    debug_pub_intensity_filter_pct_.publish(pct_msg);
    pct_msg.data = 100.0f*range_filter_pct/static_cast<float>(cloud_in->points.size());
    debug_pub_range_filter_pct_.publish(pct_msg);
    pct_msg.data = 100.0f*rover_filter_pct/static_cast<float>(cloud_in->points.size());
    debug_pub_rover_filter_pct_.publish(pct_msg);
    pct_msg.data = 100.0f*(intensity_filter_pct + range_filter_pct + rover_filter_pct)/static_cast<float>(cloud_in->points.size());
    debug_pub_total_filter_pct_.publish(pct_msg);
}

void CloudFilter::filterRangeAndIntensityVectors(std::vector<float>& ranges, std::vector<float>& intensities, std::vector<float>& angles) 
{
    // Sort the ranges and intensities based on the angles
    std::vector<std::pair<float, std::pair<float, float>>> angles_ranges_intensities;
    for (size_t i = 0; i < ranges.size(); ++i) 
    {
        angles_ranges_intensities.emplace_back(std::make_pair(angles[i], std::make_pair(ranges[i], intensities[i])));
    }
    std::sort(angles_ranges_intensities.begin(), angles_ranges_intensities.end(), 
    [](const std::pair<float, std::pair<float, float>>& a, const std::pair<float, std::pair<float, float>>& b)
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

const bool CloudFilter::filterRoverPoints(const Eigen::Vector4f& p) 
{
    // If inside a 2D box that surrounds the rover in XY plane, filter out as well
    if (p.x() > negative_range_.x && p.x() < positive_range_.x && p.y() > negative_range_.y && p.y() < positive_range_.y) 
    {
        return true;
    }
    return false;
}

const bool CloudFilter::filterRange(const Eigen::Vector4f& p) 
{
    return p.head<2>().norm() > max_xy_range_;
}

const bool CloudFilter::filterIntensity(const float intensity) 
{ 
    return intensity < min_intensity_;
}
