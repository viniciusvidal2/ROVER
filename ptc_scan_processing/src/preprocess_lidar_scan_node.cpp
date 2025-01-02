#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include "scan_preprocessor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "preprocess_lidar_scan_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");
  ROS_INFO("Initialyzing lidar scan preprocessing node ...");

  // Reading parameters
  float rover_lengh, rover_width, livox_height_from_floor;
  np.param("/vehicle_region_box/lengh", rover_lengh, 0.6f);
  np.param("/vehicle_region_box/width", rover_width, 0.6f);
  np.param("/vehicle_region_box/height", livox_height_from_floor, 0.9f);
  float x_lidar_body, y_lidar_body, z_lidar_body, roll_lidar_body, pitch_lidar_body, yaw_lidar_body;
  std::string in_frame, out_frame;
  np.param("/lidar_extrinsics/lidar_frame", in_frame, static_cast<std::string>("livox_frame"));
  np.param("/lidar_extrinsics/vehicle_frame", out_frame, static_cast<std::string>("body"));
  np.param("/lidar_extrinsics/x_lidar_vehicle", x_lidar_body, 0.0f);
  np.param("/lidar_extrinsics/y_lidar_vehicle", y_lidar_body, 0.0f);
  np.param("/lidar_extrinsics/z_lidar_vehicle", z_lidar_body, 0.0f);
  np.param("/lidar_extrinsics/roll_lidar_vehicle", roll_lidar_body, 0.0f);   // [degrees]
  np.param("/lidar_extrinsics/pitch_lidar_vehicle", pitch_lidar_body, 0.0f); // [degrees]
  np.param("/lidar_extrinsics/yaw_lidar_vehicle", yaw_lidar_body, 0.0f);     // [degrees]

  // Print parameters
  ROS_INFO("ROVER length: %.2f meters", rover_lengh);
  ROS_INFO("ROVER width: %.2f meters", rover_width);
  ROS_INFO("Filter height from floor: %.2f meters", livox_height_from_floor);
  ROS_INFO("Relative pose: %s -> %s", in_frame.c_str(), out_frame.c_str());
  ROS_INFO("x: %.2f, y: %.2f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", x_lidar_body, y_lidar_body, z_lidar_body, roll_lidar_body, pitch_lidar_body, yaw_lidar_body);

  // Create a map to store the parameters
  std::unordered_map<std::string, float> params;
  params["rover_lengh"] = rover_lengh;
  params["rover_width"] = rover_width;
  params["livox_height_from_floor"] = livox_height_from_floor;
  params["x_lidar_body"] = x_lidar_body;
  params["y_lidar_body"] = y_lidar_body;
  params["z_lidar_body"] = z_lidar_body;
  params["roll_lidar_body"] = roll_lidar_body;
  params["pitch_lidar_body"] = pitch_lidar_body;
  params["yaw_lidar_body"] = yaw_lidar_body;
  std::unordered_map<std::string, std::string> frames;
  frames["in_frame"] = in_frame;
  frames["out_frame"] = out_frame;

  // Create the object
  ScanPreprocessor scan_preprocessor(nh, params, frames);

  ROS_INFO("Listening to sensors data ...");
  ros::spin();

  return 0;
}
