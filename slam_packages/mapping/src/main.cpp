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
    // Wait some time to let the system start
    ros::Duration(2.0).sleep();
    // If map folder already exists, create another one with the same name plus _1, not to overwrite the existing one
    std::string map_folder = "", map_name = "";
    nh.param("/map_data/save_relative_path", map_folder, static_cast<std::string>("maps"));
    nh.param("/map_data/map_name", map_name, static_cast<std::string>("map"));
    std::string map_path = std::string(std::getenv("HOME")) + "/" + map_folder + "/" + map_name;
    if (FileManipulation::directoryExists(map_path))
    {
        int i = 1;
        while (FileManipulation::directoryExists(map_path + "_" + std::to_string(i)))
        {
            i++;
        }
        map_path = map_path + "_" + std::to_string(i);
    }
    // Create a folder to store the recorded map
    FileManipulation::createDirectory(map_path);
    
    // Create the node and spin
    std::shared_ptr<MapDataSaver> node = std::make_shared<MapDataSaver>(nh, map_path);
    ros::spin();
    ros::shutdown();
    return 0;
}
