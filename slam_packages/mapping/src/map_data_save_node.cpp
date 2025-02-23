#include "mapping/map_data_save_node.h"

MapDataSaver::MapDataSaver(ros::NodeHandle &nh)
{
    // Parameters
    std::string map_name;
    ros::NodeHandle pnh("~");
    pnh.param("/debug/enable", debug_, false);
    pnh.param("/map_data/save_relative_path", folder_save_path_, static_cast<std::string>("maps"));
    pnh.param("/map_data/map_name", map_name, static_cast<std::string>("map"));
    pnh.param("/mapping/cloud_save_interval", cloud_save_interval_, 10);
    pnh.param("/mapping/min_counter_to_account_for_velocity", min_counter_to_account_for_velocity_, 100);
    pnh.param("/mapping/min_velocity_to_count_as_movement", min_velocity_to_count_as_movement_, 0.1f);

    // Create a folder to store the recorded map
    folder_save_path_ = std::string(std::getenv("HOME")) + "/" + folder_save_path_ + "/" + map_name;
    FileManipulation::createDirectory(folder_save_path_);

    // Create the txt file to save the poses received by odometry
    odometry_file_path_ = folder_save_path_ + "/odometry_positions.txt";
    FileManipulation::createTextFile(odometry_file_path_, "tx ty tz\n");

    // Create the txt file to save the GPS data plus the IMU data for poses
    gps_imu_poses_file_path_ = folder_save_path_ + "/gps_imu_poses.txt";
    FileManipulation::createTextFile(gps_imu_poses_file_path_, "lat lon alt y\n");

    // Initialize the point cloud to save the map
    cloud_map_frame_ = pcl::PointCloud<PointOut>::Ptr(new pcl::PointCloud<PointOut>);

    // Compass subscriber will be used to get the yaw angle
    compass_subscription_ = nh.subscribe<std_msgs::Float64>(
        "/mavros/global_position/compass_hdg",
        10,
        &MapDataSaver::compassCallback, this);

    // Initialize synchronized subscribers
    pointcloud_sub_.subscribe(nh, "/lidar_odometry/cloud_registered_body", 10);
    gps_sub_.subscribe(nh, "/mavros/global_position/global", 10);
    odom_sub_.subscribe(nh, "/lidar_odometry/odometry", 10);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(50), pointcloud_sub_, gps_sub_, odom_sub_));
    sync_->registerCallback(boost::bind(&MapDataSaver::mappingCallback, this, _1, _2, _3));

    // Initialize the last odometry message
    last_odom_ = Eigen::Matrix4f::Identity();
    last_odom_time_ = ros::Time::now();
    current_odom_time_ = ros::Time::now();
}

void MapDataSaver::compassCallback(const std_msgs::Float64::ConstPtr &msg)
{
    // Invert the yaw based on Ardupilot convention that clockwise is positive
    current_compass_yaw_ = (90.0 - msg->data) * M_PI / 180.0; // [RAD]
    // Make sure the yaw is in the range -M_PI to M_PI
    if (current_compass_yaw_ > M_PI)
    {
        current_compass_yaw_ -= 2 * M_PI;
    }
    else if (current_compass_yaw_ < -M_PI)
    {
        current_compass_yaw_ += 2 * M_PI;
    }
}

void MapDataSaver::mappingCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                                   const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                   const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    // Convert the odometry message to a homogeneous matrix
    Eigen::Matrix4f odom_T_lidar = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf odom_q_lidar(odom_msg->pose.pose.orientation.w,
                                    odom_msg->pose.pose.orientation.x,
                                    odom_msg->pose.pose.orientation.y,
                                    odom_msg->pose.pose.orientation.z);
    Eigen::Vector3f odom_t_lidar(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    odom_T_lidar.block<3, 3>(0, 0) = odom_q_lidar.toRotationMatrix();
    odom_T_lidar.block<3, 1>(0, 3) = odom_t_lidar;

    // Get the incoming point cloud with intensity information and convert to RGB format in R channel
    pcl::PointCloud<PointIn>::Ptr i_cloud = pcl::PointCloud<PointIn>::Ptr(new pcl::PointCloud<PointIn>);
    pcl::fromROSMsg(*pointcloud_msg, *i_cloud);
    pcl::PointCloud<PointOut>::Ptr rgb_cloud = pcl::PointCloud<PointOut>::Ptr(new pcl::PointCloud<PointOut>);
    rgb_cloud->points.reserve(i_cloud->size());
    for (const auto& pi : i_cloud->points)
    {
        PointOut prgb;
        prgb.x = pi.x;
        prgb.y = pi.y;
        prgb.z = pi.z;
        prgb.r = static_cast<std::uint8_t>(pi.intensity);
        rgb_cloud->points.emplace_back(prgb);
    }

    // Transform and add the point cloud to the map
    pcl::transformPointCloud(*rgb_cloud, *rgb_cloud, odom_T_lidar);
    *cloud_map_frame_ += *rgb_cloud;
    ++cloud_counter_;

    // Calculate velocity
    const Eigen::Vector3f current_position(odom_T_lidar.block<3, 1>(0, 3));
    const Eigen::Vector3f previous_position(last_odom_.block<3, 1>(0, 3));
    current_odom_time_ = odom_msg->header.stamp;
    const float dt = (current_odom_time_ - last_odom_time_).toSec();
    const float velocity = (current_position - previous_position).norm() / dt;
    last_odom_ = odom_T_lidar;
    last_odom_time_ = current_odom_time_;

    // Do not save if conditions are not met
    if (cloud_counter_ > min_counter_to_account_for_velocity_ &&
        velocity < min_velocity_to_count_as_movement_)
    {
        ROS_WARN("Velocity is too low to save the map. Velocity: %f", velocity);
        return;
    }

    // Save the point cloud tile if the counter reaches the save interval
    if (cloud_counter_ % cloud_save_interval_ == 0)
    {
        std::string cloud_file_path = folder_save_path_ + "/cloud_" + std::to_string(cloud_counter_) + ".pcd";
        pcl::io::savePCDFileBinary(cloud_file_path, *cloud_map_frame_);
        if (debug_)
        {
            ROS_INFO("Saved cloud %d", cloud_counter_);
        }
        cloud_map_frame_->clear();
    }

    // Write a line to the odometry file
    // It should be tx ty tz
    std::ofstream odom_positions_file(odometry_file_path_, std::ios::app);
    odom_positions_file << current_position.x() << " "
                        << current_position.y() << " "
                        << current_position.z() << std::endl;
    odom_positions_file.close();

    // Write a line to the GPS IMU file
    // It should be lat lon alt yaw
    std::ofstream gps_imu_poses_file(gps_imu_poses_file_path_, std::ios::app);
    gps_imu_poses_file << std::fixed << std::setprecision(8)
                       << gps_msg->latitude << " "
                       << gps_msg->longitude << " "
                       << gps_msg->altitude << " "
                       << current_compass_yaw_ << std::endl;
    gps_imu_poses_file.close();
}

void MapDataSaver::onShutdown()
{
    // Save the final point cloud, if we have one left that was not saved
    if (cloud_map_frame_->size() > 0)
    {
        std::string cloud_file_path = folder_save_path_ + "/cloud_" + std::to_string(cloud_counter_) + ".pcd";
        pcl::io::savePCDFileBinary(cloud_file_path, *cloud_map_frame_);
        if (debug_)
        {
            ROS_INFO("Saved cloud %d", cloud_counter_);
        }
    }
}
