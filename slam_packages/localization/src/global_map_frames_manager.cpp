#include "localization/global_map_frames_manager.h"

GlobalMapFramesManager::GlobalMapFramesManager(const std::string data_folder, const std::string map_name, const std::size_t num_poses_max)
    : num_poses_max_(num_poses_max)
{
    map_path_ = data_folder + "/" + map_name;
}

std::vector<Eigen::Vector3d> GlobalMapFramesManager::loadOdometryPositions(const std::string &odom_positions_file) const
{
    std::vector<Eigen::Vector3d> odom_positions;
    std::ifstream file(odom_positions_file);
    if (!file.is_open())
    {
        std::cerr << "Error opening file " << odom_positions_file << std::endl;
        return odom_positions;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // Skip first line
        if (line == "tx ty tz")
        {
            continue;
        }
        std::istringstream iss(line);
        Eigen::Vector3d pos;
        iss >> pos.x() >> pos.y() >> pos.z();
        odom_positions.push_back(pos);
    }

    return odom_positions;
}

std::vector<Eigen::Vector3d> GlobalMapFramesManager::loadGlobalInfo(const std::string &gps_yaw_file)
{
    std::vector<Eigen::Vector3d> latlonalt_vector;
    std::ifstream file(gps_yaw_file);
    if (!file.is_open())
    {
        std::cerr << "Error opening file " << gps_yaw_file << std::endl;
        return latlonalt_vector;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // Skip first line
        if (line == "lat lon alt y")
        {
            continue;
        }
        std::istringstream iss(line);
        Eigen::Vector3d latlonalt;
        float yaw;
        iss >> latlonalt.x() >> latlonalt.y() >> latlonalt.z() >> yaw;
        latlonalt_vector.push_back(latlonalt);

        // Save to the GPS table
        if (latlonalt.z() > 0)
        {
            gps_altitude_table_.push_back(latlonalt);
        }
    }

    return latlonalt_vector;
}

float GlobalMapFramesManager::getClosestAltitude(const double lat, const double lon) const
{
    // Check if the table is empty
    if (gps_altitude_table_.empty())
    {
        return 0.0f;
    }

    // Find the closest latitude and longitude and return the correspondent altitude
    double min_dist = std::numeric_limits<double>::max();
    float match_altitude = 0.0f;
    for (auto &latlonalt : gps_altitude_table_)
    {
        const double dist = std::sqrt(std::pow(lat - latlonalt.x(), 2) + std::pow(lon - latlonalt.y(), 2));
        if (dist < min_dist)
        {
            min_dist = dist;
            match_altitude = latlonalt.z();
        }
    }

    return match_altitude;
}

pcl::PointCloud<PointT>::Ptr GlobalMapFramesManager::getMapCloud(const float voxel_size) const
{
    // Check if the map_path_ exists with a built map, and if so just load it and return
    // If not, lets merge the scans and save it for next iterations
    std::string map_cloud_path = map_path_ + "/map.pcd";
    if (access(map_cloud_path.c_str(), F_OK) != -1)
    {
        pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile<PointT>(map_cloud_path, *map_cloud);
        return map_cloud;
    }
    else
    {
        return mergeScansAndSave(voxel_size);
    }
}

pcl::PointCloud<PointT>::Ptr GlobalMapFramesManager::mergeScansAndSave(const float voxel_size) const
{
    pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>);
    // Look in the folder for all pcd files, and add them to the map cloud
    DIR *dir;
    struct dirent *ent;

    // Open the directory to look for all pcd files we need to create a map
    if ((dir = opendir(map_path_.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            std::string file_name = ent->d_name;
            if (file_name.size() > 4 && file_name.substr(file_name.size() - 4) == ".pcd")
            {
                // Concatenate the root folder with the file name
                std::string full_path = map_path_ + "/" + file_name;
                // Load the point cloud
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                pcl::io::loadPCDFile<PointT>(full_path, *cloud);
                // Concatenate the clouds
                *map_cloud += *cloud;
            }
        }
        closedir(dir);
    }
    else
    {
        auto error_msg = "Could not open " + map_path_ + " directory";
        perror(error_msg.c_str());
        return map_cloud;
    }

    if (map_cloud->empty())
    {
        std::cerr << "Error: no point clouds to merge!" << std::endl;
        return map_cloud;
    }

    // Downsample the map cloud
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(map_cloud);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*map_cloud);
    // Save the map cloud
    pcl::io::savePCDFileBinary(map_path_ + "/map.pcd", *map_cloud);

    return map_cloud;
}

bool GlobalMapFramesManager::filterBadReadings(std::vector<Eigen::Vector3d> &odom_positions,
                                               std::vector<Eigen::Vector3d> &latlonalt) const
{
    // Check if the sizes of the vectors are the same
    if (odom_positions.size() != latlonalt.size())
    {
        std::cerr << "Error: the sizes of the vectors are not the same!" << std::endl;
        return false;
    }

    // Filter the bad readings
    std::vector<Eigen::Vector3d> odom_positions_filtered;
    std::vector<Eigen::Vector3d> latlonalt_filtered;
    for (size_t i = 0; i < odom_positions.size(); ++i)
    {
        if (latlonalt[i].z() < 0)
        {
            continue;
        }
        odom_positions_filtered.push_back(odom_positions[i]);
        latlonalt_filtered.push_back(latlonalt[i]);
    }

    // Update the vectors
    odom_positions = odom_positions_filtered;
    latlonalt = latlonalt_filtered;

    // Check from the valid odom points if we have traveled more than 3 meters
    // This is to ensure that we have enough data to compute the transform
    double traveled_distance = 0.0;
    for (size_t i = 1; i < odom_positions.size(); ++i)
    {
        traveled_distance += (odom_positions[i] - odom_positions[i - 1]).norm();
    }
    if (traveled_distance < 3.0)
    {
        std::cerr << "Error: not enough traveled distance!" << std::endl;
        return false;
    }

    return true;
}

Eigen::Matrix4d GlobalMapFramesManager::getMapTGlobal()
{
    // Load the odometry positions and the global info
    std::string odom_positions_file = map_path_ + "/odometry_positions.txt";
    std::string gps_yaw_file = map_path_ + "/gps_imu_poses.txt";
    std::vector<Eigen::Vector3d> odom_positions = loadOdometryPositions(odom_positions_file);
    std::vector<Eigen::Vector3d> latlonalt = loadGlobalInfo(gps_yaw_file);
    if (odom_positions.empty() || latlonalt.empty())
    {
        std::cerr << "Error: no valid odometry or global info data!" << std::endl;
        return Eigen::Matrix4d::Identity();
    }
    if (!filterBadReadings(odom_positions, latlonalt))
    {
        std::cerr << "Error: filtering bad readings!" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    // Grab the odom and gps reading at each 0.5 meters to be processed
    std::vector<Eigen::Vector3d> odom_positions_downsampled;
    std::vector<Eigen::Vector3d> utm_downsampled;
    double traveled_distance = 0.0;
    for (size_t i = 1; i < odom_positions.size(); ++i)
    {
        traveled_distance += (odom_positions[i] - odom_positions[i - 1]).norm();
        if (traveled_distance > 0.5)
        {
            // Convert GPS points to UTM coordinates to work in meters in both frames
            double utm_northing, utm_easting;
            double lat = latlonalt[i].x(), lon = latlonalt[i].y(), alt = latlonalt[i].z();
            UTM::LLtoUTM(lat, lon, utm_northing, utm_easting);
            utm_downsampled.emplace_back(utm_easting, utm_northing, alt);
            odom_positions_downsampled.push_back(odom_positions[i]);
            traveled_distance = 0.0;
        }
    }
    if (odom_positions_downsampled.size() < 4)
    {
        std::cerr << "Error: not enough downsampled odom positions!" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    return computeMapTGlobal(utm_downsampled, odom_positions_downsampled);
}

Eigen::Matrix4d GlobalMapFramesManager::computeMapTGlobal(const std::vector<Eigen::Vector3d> &gps_points,
                                                          const std::vector<Eigen::Vector3d> &odom_points)
{
    // Compute centroids
    Eigen::Vector3d centroid_gps = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_odom = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < gps_points.size(); ++i)
    {
        centroid_gps += gps_points[i];
        centroid_odom += odom_points[i];
    }
    centroid_gps /= gps_points.size();
    centroid_odom /= odom_points.size();

    // Center the points
    Eigen::MatrixXd gps_centered(3, gps_points.size());
    Eigen::MatrixXd odom_centered(3, odom_points.size());

    for (size_t i = 0; i < gps_points.size(); ++i)
    {
        gps_centered.col(i) = gps_points[i] - centroid_gps;
        odom_centered.col(i) = odom_points[i] - centroid_odom;
    }

    // Compute cross-covariance matrix
    Eigen::Matrix3d H = gps_centered * odom_centered.transpose();

    // Compute SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Compute rotation
    Eigen::Matrix3d map_R_global = V * U.transpose();

    // Ensure a right-handed coordinate system
    if (map_R_global.determinant() < 0)
    {
        V.col(2) *= -1;
        map_R_global = V * U.transpose();
    }

    // Compute translation
    Eigen::Vector3d map_t_global = centroid_odom - map_R_global * centroid_gps;

    // Construct transformation matrix
    Eigen::Matrix4d map_T_global = Eigen::Matrix4d::Identity();
    map_T_global.block<3, 3>(0, 0) = map_R_global;
    map_T_global.block<3, 1>(0, 3) = map_t_global;

    return map_T_global;
}
