#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>

#include <ros/ros.h>

#include "localization/obstacle_point_cloud_generator_node.h"

inline static bool directoryExists(const std::string &path)
{
    struct stat info;
    return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_point_cloud_generator_node");
    ros::NodeHandle nh;
    // If map folder does not exist, do not run localization
    std::string map_folder = "", map_name = "";
    nh.param("/map_data/home_relative_path", map_folder, static_cast<std::string>("maps"));
    nh.param("/map_data/map_name", map_name, static_cast<std::string>("map"));
    std::string map_path = std::string(std::getenv("HOME")) + "/" + map_folder + "/" + map_name;
    if (!directoryExists(map_path))
    {
        ROS_WARN("Map with name %s does not exist yet, ending obstacle point cloud generator node ...", map_path.c_str());
        return 0;
    }
    ObstaclePointCloudGeneratorNode node(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}
