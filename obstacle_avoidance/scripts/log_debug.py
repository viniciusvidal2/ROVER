#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import contextily as ctx
import matplotlib.pyplot as plt
from time import time
import datetime


def getCurrentTimeAsString():
    """
    Returns the current time as a formatted string suitable for filenames.

    The format is: YYYYMMDD_HHMMSS
    """
    # Convert time in seconds to a Python datetime object
    current_time_dt = datetime.datetime.fromtimestamp(time())

    return current_time_dt.strftime('%Y%m%d_%H%M%S')


def createObstaclesDebugMarkerArray(obstacles):
    marker_array = MarkerArray()
    for i in range(len(obstacles)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = obstacles[i][0]
        marker.pose.position.y = obstacles[i][1]
        marker.pose.position.z = 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)

    return marker_array

def createGoalGuidedPointDebugMarkerArray(goal, guided_point):
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = goal[0]
    marker.pose.position.y = goal[1]
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker_array.markers.append(marker)
    
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = guided_point[0]
    marker.pose.position.y = guided_point[1]
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker_array.markers.append(marker)
    
    return marker_array


def createForcesDebugMarkerArray(attraction_force, repulsive_force, total_force):
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(attraction_force[0], attraction_force[1], 0))
    marker_array.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 1
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(repulsive_force[0], repulsive_force[1], 0))
    marker_array.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 2
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(total_force[0], total_force[1], 0))
    marker_array.markers.append(marker)

    return marker_array


def plotPointsOnMap(goal, guided_point, obstacles, filename):
    """
    Plots a goal, a guided point, and a list of obstacles on a map with a high zoom level.

    Parameters:
    goal (numpy.ndarray): The goal point (latitude, longitude).
    guided_point (numpy.ndarray): The guided point (latitude, longitude).
    obstacles (list of numpy.ndarray): A list of obstacle points (latitude, longitude).
    filename (str): The name of the file to save the plotted map as a PNG image.
    """

    # Create a figure and axis
    _, ax = plt.subplots(figsize=(10, 10))

    # Plot the points on the map
    ax.scatter(goal[1], goal[0], c='green', s=100, label='Goal')
    ax.scatter(guided_point[1], guided_point[0],
               c='blue', s=100, label='Guided Point')

    # Convert the list of points to a numpy array for easier indexing
    obstacles_array = np.array(obstacles)
    ax.scatter(obstacles_array[:, 1], obstacles_array[:,
               0], c='red', s=50, label='Obstacles')

    # Define the zoom level
    zoom_radius = 100 / 111320
    min_lat = min(goal[0], guided_point[0], *
                  obstacles_array[:, 0]) - zoom_radius
    max_lat = max(goal[0], guided_point[0], *
                  obstacles_array[:, 0]) + zoom_radius
    min_lon = min(goal[1], guided_point[1], *
                  obstacles_array[:, 1]) - zoom_radius
    max_lon = max(goal[1], guided_point[1], *
                  obstacles_array[:, 1]) + zoom_radius

    # Set the map limits to zoom into the area
    ax.set_xlim(min_lon, max_lon)
    ax.set_ylim(min_lat, max_lat)

    # Add basemap from contextily
    ctx.add_basemap(ax, crs='EPSG:4326', source=ctx.providers.OpenStreetMap.Mapnik)

    # Add a legend
    ax.legend()

    # Set labels
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')

    # Save the plot as a PNG file
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close()
