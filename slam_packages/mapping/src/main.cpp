#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>
#include <ros/ros.h>

#include "mapping/map_data_save_node.h"
#include "mapping/file_manipulation.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "map_data_save_node");
    ros::NodeHandle nh("~");
    // Wait some time to let the system start, especially localization node
    ros::Duration(2.0).sleep();
    // If map folder already exists, do not run mapping
    std::string map_folder = "", map_name = "";
    nh.param("/map_data/save_relative_path", map_folder, static_cast<std::string>("maps"));
    nh.param("/map_data/map_name", map_name, static_cast<std::string>("map"));
    std::string map_path = std::string(std::getenv("HOME")) + "/" + map_folder + "/" + map_name;
    if (FileManipulation::directoryExists(map_path))
    {
        ROS_WARN("Map with name %s already exists, exiting mapping node ...", map_path.c_str());
        return 0;
    }
    std::shared_ptr<MapDataSaver> node = std::make_shared<MapDataSaver>(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}
