#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from time import time
import datetime
import tf


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
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = obstacles[i][0]
        marker.pose.position.y = obstacles[i][1]
        marker.pose.position.z = 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 1.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(2)
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
    marker.scale.x = 0.7
    marker.scale.y = 0.9
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration(2)
    marker_array.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 1
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = guided_point[0]
    marker.pose.position.y = guided_point[1]
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 1.5
    marker.scale.z = 0.5
    marker.color.a = 0.7
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.lifetime = rospy.Duration(2)
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
    marker.lifetime = rospy.Duration(2)
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
    marker.lifetime = rospy.Duration(2)
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
    marker.lifetime = rospy.Duration(2)
    marker_array.markers.append(marker)

    return marker_array


def createRobotPathAreaMarker(height, width, angle):
    # Create rotated rectangle from the input data and return the marker
    # The rectagle has a tip at the origin and is rotated by the given angle
    quat = tf.transformations.quaternion_from_euler(0, 0, angle)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.scale.x = height
    marker.scale.y = width
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.lifetime = rospy.Duration(2)

    return marker
