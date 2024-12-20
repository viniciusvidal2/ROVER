#include "scan_preprocessor.h"

ScanPreprocessor::ScanPreprocessor(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params,
                                   std::unordered_map<std::string, std::string> &frames)
{
    // Subscribe to the input scan
    scan_sub_ = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 10, &ScanPreprocessor::scanCallback, this);
    // Advertise the preprocessed scan
    out_scan_pub_ = nh.advertise<livox_ros_driver2::CustomMsg>("/livox/lidar_filtered", 10);

    // Input and output frames variables
    in_frame_ = frames["in_frame"];
    out_frame_ = frames["out_frame"];
    const float x_lidar_body_ = params["x_lidar_body"];
    const float y_lidar_body_ = params["y_lidar_body"];
    const float z_lidar_body_ = params["z_lidar_body"];
    const float roll_lidar_body_ = params["roll_lidar_body"] * M_PI / 180.0f;   // [rad]
    const float pitch_lidar_body_ = params["pitch_lidar_body"] * M_PI / 180.0f; // [rad]
    const float yaw_lidar_body_ = params["yaw_lidar_body"] * M_PI / 180.0f;     // [rad]
    // Create homogeneous matrix from the relative pose
    Eigen::Matrix3f lidar_R_body = (Eigen::AngleAxisf(roll_lidar_body_, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch_lidar_body_, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw_lidar_body_, Eigen::Vector3f::UnitZ())).toRotationMatrix();
    lidar_T_body_ << lidar_R_body(0, 0), lidar_R_body(0, 1), lidar_R_body(0, 2), x_lidar_body_,
        lidar_R_body(1, 0), lidar_R_body(1, 1), lidar_R_body(1, 2), y_lidar_body_,
        lidar_R_body(2, 0), lidar_R_body(2, 1), lidar_R_body(2, 2), z_lidar_body_,
        0.0, 0.0, 0.0, 1.0;

    // Set the filter parameters
    const float rover_lengh = params["rover_lengh"];
    const float rover_width = params["rover_width"];
    const float livox_height_from_floor = params["livox_height_from_floor"];
    // The offsets are in the rover body frame
    // X forward, Y left, Z up
    negative_range_ << -rover_lengh / 2.0f, -rover_width / 2.0f, -livox_height_from_floor;
    positive_range_ << rover_lengh / 2.0f, rover_width / 2.0f, livox_height_from_floor;
}

// Scan callback
void ScanPreprocessor::scanCallback(const livox_ros_driver2::CustomMsgConstPtr &scan_msg,
                                    const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    // Output scan
    livox_ros_driver2::CustomMsg scan_out;
    scan_out.header.stamp = scan_msg->header.stamp;
    scan_out.header.frame_id = scan_msg->header.frame_id;
    scan_out.timebase = scan_msg->timebase;
    scan_out.point_num = scan_msg->point_num;
    scan_out.rsvd = scan_msg->rsvd;

    // If empty scan, just publish the incoming message
    if (scan_msg->points.empty())
    {
        out_scan_pub_.publish(scan_msg);
        return;
    }

    // Work on the input scan
    scan_out.points.reserve(scan_msg->points.size());
    for (const auto &p : scan_msg->points)
    {
        // Apply the relative pose transformation
        const Eigen::Vector4f p_in(p.x, p.y, p.z, 1.0);
        const Eigen::Vector4f p_out(lidar_T_body_ * p_in);

        // Filter if the point is inside the rover theoretical box
        if (p_out.x() > negative_range_.x() && p_out.x() < positive_range_.x() &&
            p_out.y() > negative_range_.y() && p_out.y() < positive_range_.y() &&
            p_out.z() > negative_range_.z() && p_out.z() < positive_range_.z())
        {
            continue;
        }

        // Add the point to the output scan
        livox_ros_driver2::CustomPoint p_out_msg;
        p_out_msg.x = p_out.x();
        p_out_msg.y = p_out.y();
        p_out_msg.z = p_out.z();
        p_out_msg.reflectivity = p.reflectivity;
        p_out_msg.tag = p.tag;
        p_out_msg.line = p.line;
        p_out_msg.offset_time = p.offset_time;
        scan_out.points.push_back(p_out_msg);
    }

    // Publish the filtered scan
    out_scan_pub_.publish(scan_out);
}
