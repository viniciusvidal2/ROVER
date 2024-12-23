#include "scan_preprocessor.h"

ScanPreprocessor::ScanPreprocessor(ros::NodeHandle &nh, std::unordered_map<std::string, float> &params,
                                   std::unordered_map<std::string, std::string> &frames)
{
    // Advertise the preprocessed scan and IMU
    out_scan_pub_ = nh.advertise<livox_ros_driver2::CustomMsg>("/livox/lidar_filtered", 10);
    out_imu_pub_ = nh.advertise<sensor_msgs::Imu>("/livox/imu_filtered", 10);

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
    lidar_q_body_ = Eigen::Quaternionf(lidar_R_body);

    // Set the filter parameters
    const float rover_lengh = params["rover_lengh"];
    const float rover_width = params["rover_width"];
    const float livox_height_from_floor = params["livox_height_from_floor"];
    // The offsets are in the rover body frame
    // X forward, Y left, Z up
    negative_range_ << -rover_lengh / 2.0f, -rover_width / 2.0f, -livox_height_from_floor;
    positive_range_ << rover_lengh / 2.0f, rover_width / 2.0f, livox_height_from_floor;

    // Initialize subscribers
    scan_sub_ = nh.subscribe("/livox/lidar", 1000, &ScanPreprocessor::scanCallback, this);
    imu_sub_ = nh.subscribe("/livox/imu", 1000, &ScanPreprocessor::imuCallback, this);
}

// Scan callback
void ScanPreprocessor::scanCallback(const livox_ros_driver2::CustomMsgConstPtr &scan_msg)
{
    // Output scan
    livox_ros_driver2::CustomMsg scan_out;
    scan_out.header = scan_msg->header;
    scan_out.timebase = scan_msg->timebase;
    scan_out.lidar_id = scan_msg->lidar_id;
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
        livox_ros_driver2::CustomPoint p_out_msg = p;
        p_out_msg.x = p_out.x();
        p_out_msg.y = p_out.y();
        p_out_msg.z = p_out.z();
        scan_out.points.push_back(p_out_msg);
    }
    scan_out.point_num = static_cast<std::uint32_t>(scan_out.points.size());

    // Publish the filtered scan
    out_scan_pub_.publish(scan_out);
}

void ScanPreprocessor::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    sensor_msgs::Imu out_imu = *msg;
    // Apply vehicle extrinsics to IMU original orientation, velocity and acceleration
    const Eigen::Quaternionf q_lidar(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    const Eigen::Quaternionf q_body = lidar_q_body_ * q_lidar;
    const Eigen::Vector3f angular_velocity_in(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    const Eigen::Vector3f angular_velocity_out(lidar_q_body_ * angular_velocity_in);
    const Eigen::Vector3f linear_acceleration_in(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    const Eigen::Vector3f linear_acceleration_out(lidar_q_body_ * linear_acceleration_in);
    out_imu.orientation.w = q_body.w();
    out_imu.orientation.x = q_body.x();
    out_imu.orientation.y = q_body.y();
    out_imu.orientation.z = q_body.z();
    out_imu.angular_velocity.x = angular_velocity_out.x();
    out_imu.angular_velocity.y = angular_velocity_out.y();
    out_imu.angular_velocity.z = angular_velocity_out.z();
    out_imu.linear_acceleration.x = linear_acceleration_out.x();
    out_imu.linear_acceleration.y = linear_acceleration_out.y();
    out_imu.linear_acceleration.z = linear_acceleration_out.z();
    // Publish output
    out_imu_pub_.publish(out_imu);
}
