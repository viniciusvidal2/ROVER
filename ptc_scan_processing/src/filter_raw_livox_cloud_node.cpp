#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include "cloud_filter.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_raw_cloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");
  ROS_INFO("Initialyzing raw point cloud filter node ...");

  // Reading parameters
  bool apply_filter;
  np.param("/raw_cloud_filter/apply_filter", apply_filter, false);
  bool filter_range, filter_intensity, filter_rover_points, debug_cloud;
  np.param("/raw_cloud_filter/filter_range", filter_range, false);
  np.param("/raw_cloud_filter/filter_intensity", filter_intensity, false);
  np.param("/raw_cloud_filter/filter_rover_points", filter_rover_points, false);
  np.param("/raw_cloud_filter/debug_cloud", debug_cloud, false);
  float rover_lengh, rover_width, livox_height_from_floor, max_xy_range;
  np.param("/raw_cloud_filter/rover_lengh", rover_lengh, -1.0f);
  np.param("/raw_cloud_filter/rover_width", rover_width, 1.0f);
  np.param("/raw_cloud_filter/livox_height_from_floor", livox_height_from_floor, -1.0f);
  np.param("/raw_cloud_filter/max_xy_range", max_xy_range, 1000.0f);
  float min_intensity;
  np.param("/raw_cloud_filter/min_intensity", min_intensity, 0.0f);
  float frontal_fov;
  np.param("/raw_cloud_filter/frontal_fov", frontal_fov, 270.0f); // [deg]
  float angle_resolution; // [deg]
  np.param("/pointcloud2scan/angle_resolution", angle_resolution, 1.0f);
  float x_in_out, y_in_out, z_in_out, roll_in_out, pitch_in_out, yaw_in_out;
  std::string in_frame, out_frame;
  np.param("/livox2scan/input_frame", in_frame, static_cast<std::string>("body"));
  np.param("/livox2scan/output_frame", out_frame, static_cast<std::string>("body"));
  np.param("/livox2scan/x", x_in_out, 0.0f);
  np.param("/livox2scan/y", y_in_out, 0.0f);
  np.param("/livox2scan/z", z_in_out, 0.0f);
  np.param("/livox2scan/roll", roll_in_out, 0.0f);
  np.param("/livox2scan/pitch", pitch_in_out, 0.0f);
  np.param("/livox2scan/yaw", yaw_in_out, 0.0f);

  // Print parameters
  ROS_INFO("Applying filter: %s", apply_filter ? "true" : "false");
  ROS_INFO("Filtering boat points: %s", filter_rover_points ? "true" : "false");
  ROS_INFO("Filtering by range: %s", filter_range ? "true" : "false");
  ROS_INFO("Filtering by intensity: %s", filter_intensity ? "true" : "false");
  ROS_INFO("Debug cloud: %s", debug_cloud ? "true" : "false");
  ROS_INFO("Boat length: %.2f meters", rover_lengh);
  ROS_INFO("Boat width: %.2f meters", rover_width);
  ROS_INFO("Max height: %.2f meters", livox_height_from_floor);
  ROS_INFO("Max XY range: %.2f meters", max_xy_range);
  ROS_INFO("Min intensity: %.2f units", min_intensity);
  ROS_INFO("Frontal FOV: %.2f degrees", frontal_fov);
  ROS_INFO("Angle resolution: %.2f degrees", angle_resolution);
  ROS_INFO("Relative pose: %s -> %s", in_frame.c_str(), out_frame.c_str());
  ROS_INFO("x: %.2f, y: %.2f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", x_in_out, y_in_out, z_in_out, roll_in_out, pitch_in_out, yaw_in_out);

  // Create a map to store the parameters
  std::unordered_map<std::string, float> params;
  params["rover_lengh"] = rover_lengh;
  params["rover_width"] = rover_width;
  params["livox_height_from_floor"] = livox_height_from_floor;
  params["max_xy_range"] = max_xy_range;
  params["min_intensity"] = min_intensity;
  params["frontal_fov"] = frontal_fov;
  params["angle_resolution"] = angle_resolution;
  params["x_in_out"] = x_in_out;
  params["y_in_out"] = y_in_out;
  params["z_in_out"] = z_in_out;
  params["roll_in_out"] = roll_in_out;
  params["pitch_in_out"] = pitch_in_out;
  params["yaw_in_out"] = yaw_in_out;
  std::unordered_map<std::string, bool> flags;
  flags["apply_filter"] = apply_filter;
  flags["filter_range"] = filter_range;
  flags["filter_rover_points"] = filter_rover_points;
  flags["filter_intensity"] = filter_intensity;
  flags["debug_cloud"] = debug_cloud;
  std::unordered_map<std::string, std::string> frames;
  frames["in_frame"] = in_frame;
  frames["out_frame"] = out_frame;

  // Create the cloud filter object
  CloudFilter cloud_filter(nh, params, flags, frames);
  
  ROS_INFO("Listening to sensors data ...");
  ros::spin();

  return 0;
}
