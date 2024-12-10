#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import State, GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
import numpy as np
from time import time
import datetime
import tf


def getCurrentTimeAsString() -> str:
    """Returns the current time as a formatted string suitable for filenames.

    Returns:
        str: The format is: YYYYMMDD_HHMMSS
    """
    # Convert time in seconds to a Python datetime object
    current_time_dt = datetime.datetime.fromtimestamp(time())

    return current_time_dt.strftime('%Y%m%d_%H%M%S')


def createObstaclesDebugMarkerArray(obstacles: list) -> MarkerArray:
    """Creates the obstacles visualization as marker arrays

    Args:
        obstacles (list): list of obstacles in baselink frame

    Returns:
        MarkerArray: markers as sheperes in baselink frame
    """
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


# def createGoalGuidedPointDebugMarkerArray(goal: np.ndarrady, guided_point: np.ndarray) -> MarkerArray:
def createGoalGuidedPointDebugMarkerArray(goal: np.ndarray, guided_point: np.ndarray) -> MarkerArray:
    """Creates the goal and the guided point marker arrays in baselink frame

    Args:
        goal (np.ndarrady): goal point in baselink frame
        guided_point (np.ndarray): guided point in baselink frame

    Returns:
        MarkerArray: marker array of spheres for the two points, with proper scales and colors
    """
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


def createForcesDebugMarkerArray(attraction_force: np.ndarray, repulsive_force: np.ndarray, total_force: np.ndarray) -> MarkerArray:
    """Creates the forces marker array in baselink frame

    Args:
        attraction_force (np.ndarray): goal attraction force
        repulsive_force (np.ndarray): resulting obstacle repulsive force
        total_force (np.ndarray): sum of previous forces

    Returns:
        MarkerArray: marker array for the forces
    """
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


def createRobotPathAreaMarker(height: float, width: float, angle: float) -> Marker:
    """Creates an array marker with the size of the path we can walk by from the calculated guided point

    Args:
        height (float): path lenth up to the guided point in baselink frame
        width (float): path width
        angle (float): angle in baselink frame [RAD]

    Returns:
        Marker: the path arrow marker
    """
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


def logCallbackLoop(obstacles_baselink_frame_xy: list, goal_baselink_frame: np.ndarray, guided_point_baselink_frame: np.ndarray,
                    current_state: State, current_waypoint_index: int, current_location: NavSatFix, 
                    current_yaw: float, current_target: GlobalPositionTarget, target_baselink: np.ndarray) -> None:
    """Generates the debug plot to keep up with the node execution in real time

    Args:
        obstacles_baselink_frame_xy (list): list of obstacles in baselink frame
        goal_baselink_frame (np.ndarray): goal point in baselink frame
        guided_point_baselink_frame (np.ndarray): guided point in baselink frame
        current_state (State): the navigation mode we are using (MANUAL, AUTO or GUIDED) in mavlink message type
        current_waypoint_index (int): index of the waypoint we are pursuing in the current AUTO mission (even if in GUIDED mode)
        current_location (NavSatFix): current GPS reading for our vehicle location
        current_yaw (float): yaw angle in world frame [RAD]
        current_target (GlobalPositionTarget): current target point in world frame
        target_baselink (np.ndarray): current target in baselink frame
    """
    rospy.loginfo(
        "=================================================================")
    rospy.loginfo(f"Information for this loop: {time()}")
    rospy.loginfo(f"Current state: {current_state.mode}")
    rospy.loginfo(f"Current waypoint index: {current_waypoint_index}")
    rospy.loginfo(
        f"Current location: lat {current_location.latitude} lon {current_location.longitude}")
    rospy.loginfo(f"Current yaw: {np.degrees(current_yaw)} degrees")
    rospy.loginfo(
        f"Original goal: lat {current_target.latitude} lon {current_target.longitude}")
    rospy.loginfo(
        f"Goal point heard from board in baselink frame: {target_baselink[0]} x, {target_baselink[1]} y")
    rospy.loginfo(
        f"Goal direction in baselink frame: {goal_baselink_frame}")
    rospy.loginfo(
        f"Guided point in baselink frame: {guided_point_baselink_frame}")
    for i, obstacle in enumerate(obstacles_baselink_frame_xy):
        rospy.loginfo(f"Obstacle {i} in baselink frame: {obstacle}")
    rospy.loginfo(
        "=================================================================")
