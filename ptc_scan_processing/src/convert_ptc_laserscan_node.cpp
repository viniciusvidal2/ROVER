#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <unordered_map>

#include "cloud_scan_converter.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_scan_converter_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");
  ROS_INFO("Initialyzing point cloud to laser scan convertion node ...");

  // Reading parameters
  float max_xy_range, min_intensity, frontal_fov, angle_resolution, vehicle_height;
  np.param("/ptc_scan_filter/max_xy_range", max_xy_range, 1000.0f);
  np.param("/ptc_scan_filter/min_intensity", min_intensity, 0.0f);
  np.param("/ptc_scan_filter/frontal_fov", frontal_fov, 270.0f); // [deg]
  np.param("/ptc_scan_converter/angle_resolution", angle_resolution, 1.0f); // [deg]
  np.param("/vehicle_region_box/height", vehicle_height, 0.9f);

  // Print parameters
  ROS_INFO("Max XY range: %.2f meters", max_xy_range);
  ROS_INFO("Min intensity: %.2f units", min_intensity);
  ROS_INFO("Frontal FOV: %.2f degrees", frontal_fov);
  ROS_INFO("Angle resolution: %.2f degrees", angle_resolution);

  // Create a map to store the parameters
  std::unordered_map<std::string, float> params;
  params["max_xy_range"] = max_xy_range;
  params["min_intensity"] = min_intensity;
  params["frontal_fov"] = frontal_fov;
  params["angle_resolution"] = angle_resolution;
  params["vehicle_height"] = vehicle_height;

  // Create the cloud scan converter object
  CloudScanConverter cloud_scan_converter(nh, params);
  
  ROS_INFO("Listening to lidar point cloud data ...");
  ros::spin();

  return 0;
}
