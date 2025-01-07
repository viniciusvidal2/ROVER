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
    // If map folder already exists, do not run mapping
    std::string map_folder = "";
    nh.param("/map_data/save_relative_path", map_folder, static_cast<std::string>("Desktop/map_data"));
    if (FileManipulation::directoryExists(map_folder))
    {
        ROS_WARN("Map with name %s already exists, exiting mapping node ...", map_folder.c_str());
        return 0;
    }
    std::shared_ptr<MapDataSaver> node = std::make_shared<MapDataSaver>(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}
