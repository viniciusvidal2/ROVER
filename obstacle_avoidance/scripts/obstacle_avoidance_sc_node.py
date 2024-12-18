#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State, GlobalPositionTarget, WaypointList, HomePosition, PositionTarget
from mavros_msgs.srv import SetMode, WaypointSetCurrent, CommandTOL
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from time import time
from typing import Tuple
from log_debug import *
from collision_lib import *
from frame_convertions import *


class ObstacleAvoidance:
    ############################################################################
    # region CONSTRUCTOR
    ############################################################################
    def __init__(self) -> None:
        rospy.init_node("obstacle_avoidance_node", anonymous=False)

        # User arguments
        self.command_sending_interval = rospy.get_param(
            '~sending_t', 0.5)  # [s]
        self.max_obstacle_distance = rospy.get_param('~max_dist', 9)  # [m]
        self.corridor_width = rospy.get_param('~corridor_width', 5)  # [m]
        self.turn_rate = rospy.get_param('~turn_rate', 30) # [degrees]

        # Control the time when we last sent a guided point
        self.last_command_time = time()
        # Variables
        self.current_yaw = 0.0  # [RAD]
        self.current_location = None  # GPS data
        self.current_state = State()  # vehicle driving mode
        self.debug_mode = True  # debug mode to print more information
        self.home_waypoint = None  # home waypoint data, contains home lat and lon
        self.waypoints_list = None  # list of waypoints in the autonomous mission
        self.current_waypoint_index = -1  # autonomous mission waypoint we are tracking
        self.current_target = None  # target waypoint data in AUTO mode
        self.angle_to_goal_max = 10  # [degrees]
        self.start_search_side = 'l'  # side to start looking for available paths, l or r
        self.previous_guided_point_angle = None  # [RAD]
        self.previous_current_turn_angle_threshold = 45.0*np.pi/180.0  # [RAD]
        self.low_pass_guided_point_angle = self.turn_rate*np.pi/180.0  # [RAD]

        # Subscribers to mavros and laserscan messages
        rospy.Subscriber("/livox/scan", LaserScan,
                         self.laserScanCallback, queue_size=1)
        rospy.Subscriber("/mavros/state", State,
                         self.stateCallback, queue_size=1)
        rospy.Subscriber("/mavros/global_position/global",
                         NavSatFix, self.gpsCallback, queue_size=1)
        rospy.Subscriber("/mavros/global_position/compass_hdg",
                         Float64, self.compassCallback, queue_size=1)
        rospy.Subscriber("/mavros/setpoint_raw/target_global",
                         GlobalPositionTarget, self.currentTargetCallback, queue_size=1)
        rospy.Subscriber("/mavros/mission/waypoints",
                         WaypointList, self.missionWaypointsCallback, queue_size=1)
        rospy.Subscriber("/mavros/home_position/home",
                         HomePosition, self.homePositionCallback, queue_size=1)

        # Publishers
        self.setpoint_global_pub = rospy.Publisher(
            '/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
        self.setpoint_local_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.obstacles_pub = rospy.Publisher(
            '/obstacle_avoidance/obstacles', MarkerArray, queue_size=1)
        self.goal_guided_point_pub = rospy.Publisher(
            '/obstacle_avoidance/goal_guided_point', MarkerArray, queue_size=1)
        self.robot_path_area_pub = rospy.Publisher(
            '/obstacle_avoidance/robot_path_area', Marker, queue_size=1)

        # Services
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/mission/set_current')
        rospy.wait_for_service('/mavros/cmd/command')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_current_wp_srv = rospy.ServiceProxy(
            '/mavros/mission/set_current', WaypointSetCurrent)
        self.command_tol_srv = rospy.ServiceProxy(
            '/mavros/cmd/command', CommandTOL)

        rospy.loginfo("Obstacle avoidance node initialized.")
        rospy.spin()

    # endregion
    ############################################################################
    # region MAVLINK CALLBACKS
    ############################################################################

    def stateCallback(self, state: State) -> None:
        """Current vehicle driving state

        Args:
            state (State): current vehicle driving state
        """
        self.current_state = state

    def gpsCallback(self, data: NavSatFix) -> None:
        """Get GPS data

        Args:
            data (NavSatFix): gps data from mavros
        """
        self.current_location = data

    def compassCallback(self, data: Float64) -> None:
        """Compass data for orientation

        Args:
            data (Float64): yaw angle, from 0~360 degrees, 0 to the north, increasing clockwise. 
        """
        self.current_yaw = np.radians(data.data)  # [RAD]

    def currentTargetCallback(self, data: GlobalPositionTarget) -> None:
        """Current target from mavros, usually the next item in the autonomous mission

        Args:
            data (GlobalPositionTarget): Next mission point in global coordinates
        """
        # We use this callback to have a reference if the mission is not updated yet
        if not self.current_target or not self.waypoints_list:
            self.current_target = data
            if self.debug_mode:
                rospy.logwarn(
                    f"Received new target point in current target callback: {self.current_target.latitude} {self.current_target.longitude}.")

    def missionWaypointsCallback(self, data: WaypointList) -> None:
        """Mission currently set in the pixhawk board

        Args:
            data (WaypointList): list of waypoints in global coordinates, with the current one marked in the properties
        """
        self.waypoints_list = data.waypoints
        if len(self.waypoints_list) == 0:
            return

        if self.home_waypoint:
            # If the last waypoint coordinates are 0, that means it is a return to launch waypoint, so we add the home waypoint values to this waypoint
            if self.waypoints_list[-1].x_lat == 0 and self.waypoints_list[-1].y_long == 0:
                self.waypoints_list[-1].x_lat = self.home_waypoint.geo.latitude
                self.waypoints_list[-1].y_long = self.home_waypoint.geo.longitude
            # Set the first point in the mission to be the home as well, to make sure we know it when coming back from a blocked region
            self.waypoints_list[0].x_lat = self.home_waypoint.geo.latitude
            self.waypoints_list[0].y_long = self.home_waypoint.geo.longitude

        if self.debug_mode:
            rospy.logwarn(
                f"Received mission waypoints in mission waypoints callback: {len(self.waypoints_list)} waypoints.")
            for i, waypoint in enumerate(self.waypoints_list):
                rospy.logwarn(
                    f"Mission waypoint {i+1}/{len(self.waypoints_list)}: {waypoint.x_lat}, {waypoint.y_long}, {waypoint.z_alt}")
                if waypoint.is_current:
                    rospy.logwarn(
                        f"Current mission waypoint: {waypoint.x_lat}, {waypoint.y_long}, {waypoint.z_alt}")

        # If we are in GUIDED mode, no need to update the current target
        if self.current_state.mode == "GUIDED":
            return

        # Get the current target waypoint if we are in a mission
        for i, waypoint in enumerate(self.waypoints_list):
            if waypoint.is_current:
                self.current_target = GlobalPositionTarget()
                self.current_target.latitude = waypoint.x_lat
                self.current_target.longitude = waypoint.y_long
                self.current_target.altitude = waypoint.z_alt
                self.current_waypoint_index = i
                if self.debug_mode:
                    rospy.logwarn(
                        f"Target point set to {self.current_target.latitude}, {self.current_target.longitude} in mission callback.")
                break

    def homePositionCallback(self, data: HomePosition) -> None:
        """Last home waypoint set by the board, before or right at the mission start

        Args:
            data (HomePosition): home position in global coordinates
        """
        # Set the home point so we know what to do if we are returning to launch
        if not self.home_waypoint:
            self.home_waypoint = data
        else:
            if self.home_waypoint.geo.latitude != data.geo.latitude or self.home_waypoint.geo.longitude != data.geo.longitude:
                self.home_waypoint = data
        if self.debug_mode and self.home_waypoint:
            rospy.logwarn(
                f"Home waypoint set to {self.home_waypoint.geo.latitude}, {self.home_waypoint.geo.longitude}")

    # endregion
    ############################################################################
    # region CONTROL FUNCTIONS
    ############################################################################
    def setFlightMode(self, mode: str) -> None:
        """Set the flight mode we want to navigate with

        Args:
            mode (str): mode name, either MANUAL, GUIDED or AUTO
        """
        # If we are already in the right mode, don't do anything
        if self.current_state.mode == mode:
            return
        try:
            response = self.set_mode_service(custom_mode=mode)
            if response.mode_sent:
                rospy.logwarn(f"Changing navigation mode to {mode}...")
                while self.current_state.mode != mode:
                    response = self.set_mode_service(custom_mode=mode)
                    rospy.logwarn(
                        "Waiting for mode change request confirmation ...")
                    rospy.sleep(0.1)
                rospy.logwarn(f"Mode {mode} activated.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to change navigation mode to {mode}: {e}")

    def setCurrentTargetToPreviousWaypoint(self) -> None:
        """Tries to return to previous waypoint if no path to the next waypoint is observed
        """
        if not self.current_waypoint_index:
            return
        # Only returning to previous point if index is larger than 2 for now
        if self.current_waypoint_index <= 2:
            if self.debug_mode:
                rospy.logwarn(
                    "Cannot set current target to previous waypoint, already at the first waypoint.")
        else:
            previous_waypoint_index = self.current_waypoint_index - 1
            try:
                response = self.set_current_wp_srv(previous_waypoint_index)
                if response.success:
                    rospy.logwarn(
                        f"Successfully set current waypoint to {previous_waypoint_index}")
                else:
                    rospy.logwarn(
                        f"Failed to set current waypoint to {previous_waypoint_index}")
            except rospy.ServiceException as e:
                rospy.logerr(
                    f"Waypoint set service call failed for index {previous_waypoint_index}: {e}")
            # Setting back the target and waypoint index
            self.current_waypoint_index = previous_waypoint_index
            self.current_target = GlobalPositionTarget()
            self.current_target.latitude = self.waypoints_list[previous_waypoint_index].x_lat
            self.current_target.longitude = self.waypoints_list[previous_waypoint_index].y_long
            self.current_target.altitude = self.waypoints_list[previous_waypoint_index].z_alt
            self.setFlightMode(mode="AUTO")

    def sendGuidedPointGlobalFrame(self, guided_point_baselink_frame: np.ndarray) -> None:
        """Receives a guided waypoint in baselink frame coordinates, transforms it to global frame and sends to the board as next goal
        while avoiding obstacles

        Args:
            guided_point_baselink_frame (np.ndarray): waypoint [x, y] in baselink frame
        """
        # Convert the travel point in baselink frame to world frame
        guided_point_world_frame_lat, guided_point_world_frame_lon = baselinkToWorld(
            xy_baselink=guided_point_baselink_frame,
            current_lat=self.current_location.latitude, current_lon=self.current_location.longitude, current_yaw=self.current_yaw)

        # Send the new point to the vehicle
        guided_point_world_frame_msg = GlobalPositionTarget()
        guided_point_world_frame_msg.header = Header()
        guided_point_world_frame_msg.header.stamp = rospy.Time.now()
        guided_point_world_frame_msg.latitude = guided_point_world_frame_lat
        guided_point_world_frame_msg.longitude = guided_point_world_frame_lon
        guided_point_world_frame_msg.altitude = self.current_location.altitude
        self.setpoint_global_pub.publish(guided_point_world_frame_msg)

    # endregion
    ############################################################################
    # region AVOIDANCE METHODS
    ############################################################################
    def directionFromShapeCollision(self, goal_baselink_frame: np.ndarray, obstacles_baselink_frame_xy: list) -> Tuple[bool, np.ndarray]:
        """Calculates the direction we must follow based on the study of possible obstacles in our original path

        Args:
            goal_baselink_frame (np.ndarray): next autonomous waypoint in mission in baselink frame [x, y]
            obstacles_baselink_frame_xy (list): list of obstacles written as [x, y] in baselink frame

        Returns:
            bool, np.ndarray: whether we found a valid guided point during the search, and its coordinate in baselink frame [x, y]
        """
        # Calculating goal characteristics
        goal_angle_baselink_frame = np.arctan2(
            goal_baselink_frame[1], goal_baselink_frame[0])

        # Calculate the angles we will test to create the best trajectory, minding the critical zone
        angle_step = 5  # [degrees]
        full_test_range = 90  # [degrees]
        angle_tests = createAngleTestSequence(
            goal_angle=np.degrees(goal_angle_baselink_frame), angle_step=angle_step, full_test_range=full_test_range, start_side=self.start_search_side)  # [RAD]
        if self.debug_mode:
            rospy.logwarn(
                f"Goal direction in baselink frame: {goal_baselink_frame}")
            rospy.logwarn(
                f"Goal distance: {np.linalg.norm(goal_baselink_frame)} m")
            rospy.logwarn(
                f"Goal angle in baselink frame: {np.degrees(goal_angle_baselink_frame)} degrees")

        # Get the best trajectory from the shapes we previously estimated
        guided_point_baselink_frame, guided_to_goal_angle = calculateBestTrajectoryGuidedPoint(
            angle_tests=angle_tests, point_distance=self.max_obstacle_distance, obstacles_baselink_frame_xy=obstacles_baselink_frame_xy,
            corridor_width=self.corridor_width)

        # # Set the preferable side to start looking for paths based on the angle we just found
        # self.start_search_side = 'l' if guided_to_goal_angle >= 0 else 'r'

        # If we have an issue finding this trajectory, we should go back to the previous waypoint
        if np.linalg.norm(guided_point_baselink_frame) == 0:
            rospy.logerr(
                "No path was found to avoid obstacles, going back to previous waypoint in mission!")
            self.setCurrentTargetToPreviousWaypoint()
            return False, np.zeros_like(guided_point_baselink_frame)

        # Start avoiding only if the angle to the goal is big enough
        if abs(guided_to_goal_angle) > self.angle_to_goal_max:
            return True, guided_point_baselink_frame
        else:
            return False, guided_point_baselink_frame

    # endregion
    ############################################################################
    # region MAIN CONTROL LOOP CALLBACK
    ############################################################################

    def laserScanCallback(self, scan) -> None:
        """ We use lidar points to define obstacle in baselink frame and find the available path that is the closest to the waypoint we should travel to.
        We work with two main frames: baselink and world
        Baselink: X forward, Y left of the vehicle. 0~360 degrees, counter-clockwise, 0 is in the back
        World: latlon, so X (lat) points up and Y (lon) points to the right. 0~360 degrees, clockwise, 0 is in positive X 
        """
        # Track callback time
        start_time = time()

        # Avoiding the callback if the conditions are not met
        if not scan.ranges or self.current_state.mode == "MANUAL" or not self.current_target or not self.current_location or not self.home_waypoint:
            return

        # Make sure the scan values are valid before doing any math
        valid_ranges = np.array(scan.ranges)
        valid_ranges[valid_ranges == 0] = 1e6
        closest_obstacle_distance = np.min(valid_ranges)
        # Calculate the target waypoint in baselink frame
        goal_baselink_frame = worldToBaselink(
            target_lat=self.current_target.latitude, target_lon=self.current_target.longitude,
            current_location_lat=self.current_location.latitude, current_location_lon=self.current_location.longitude,
            current_yaw=self.current_yaw)
        goal_distance = np.linalg.norm(goal_baselink_frame)  # [m]
        goal_angle = np.degrees(np.arctan2(
            goal_baselink_frame[1], goal_baselink_frame[0]))  # [degrees]

        # If we are in AUTO mode we must reset the previous guided point angle
        if self.current_state.mode == "AUTO":
            self.previous_guided_point_angle = None

        # If any point is close enough, process the avoidance behavior
        if closest_obstacle_distance < 2*self.max_obstacle_distance:
            if self.debug_mode:
                rospy.logwarn(
                    f"Obstacle detected in less than {self.max_obstacle_distance}m!")

            # If we are closer to the goal than to the obstacle, just continue to it
            if goal_distance < closest_obstacle_distance:
                self.setFlightMode(mode="AUTO")
                return

            # Isolate the readings that return the obstacles in baselink frame
            obstacles_baselink_frame_ra = [[r, i * scan.angle_increment + scan.angle_min]
                                           for i, r in enumerate(valid_ranges) if r < 2*self.max_obstacle_distance]
            obstacles_baselink_frame_xy = [laserScanToXY(
                range=r, angle=a) for r, a in obstacles_baselink_frame_ra]

            # If we are roughly pointing to the goal and the FOV is safe, try to change the mode depending on the last command time
            if abs(goal_angle) < self.angle_to_goal_max and \
                checkSafeFOV(obstacles_baselink_frame_ra=obstacles_baselink_frame_ra, goal_angle_baselink_frame=np.radians(goal_angle)) and \
                    time() - self.last_command_time > self.command_sending_interval and self.current_state.mode != "AUTO":
                self.setFlightMode(mode="AUTO")
                self.last_command_time = time()
                return

            # Run the shape collision avoidance methodology
            guided_point_found, guided_point_baselink_frame = self.directionFromShapeCollision(goal_baselink_frame=goal_baselink_frame,
                                                                                               obstacles_baselink_frame_xy=obstacles_baselink_frame_xy)
            if guided_point_found:
                # Check the amunt we must turn in comparison to what there was in the last time we sent a guided command
                # Apply a simple threshold low pass filter to the guided point to avoid big turns then
                guided_point_angle = np.arctan2(
                    guided_point_baselink_frame[1], guided_point_baselink_frame[0])
                if self.previous_guided_point_angle:
                    if abs(guided_point_angle - self.previous_guided_point_angle) > self.previous_current_turn_angle_threshold:
                        guided_point_angle = self.low_pass_guided_point_angle if guided_point_angle > 0 else - \
                            self.low_pass_guided_point_angle
                        guided_point_baselink_frame = np.linalg.norm(
                            guided_point_baselink_frame)*np.array([np.cos(guided_point_angle), np.sin(guided_point_angle)])
                self.previous_guided_point_angle = guided_point_angle
                # Lets only set to GUIDED and send a new point if we found it and there is enough time
                if time() - self.last_command_time > self.command_sending_interval:
                    self.setFlightMode(mode="GUIDED")
                    self.sendGuidedPointGlobalFrame(
                        guided_point_baselink_frame=guided_point_baselink_frame)
                    self.last_command_time = time()

            if self.debug_mode:
                # Log the complete loop information
                logCallbackLoop(
                    obstacles_baselink_frame_xy=obstacles_baselink_frame_xy, goal_baselink_frame=goal_baselink_frame,
                    guided_point_baselink_frame=guided_point_baselink_frame,
                    current_state=self.current_state, current_location=self.current_location,
                    current_yaw=self.current_yaw, current_target=self.current_target,
                    current_waypoint_index=self.current_waypoint_index, target_baselink=goal_baselink_frame)
                end_time = time()
                rospy.logwarn(
                    f"Time to process avoidance: {1000*(end_time - start_time)} milliseconds.")

        elif self.current_state.mode == "GUIDED":
            # Calculate the guided point to send and smooth the transition to auto according to angle
            if abs(goal_angle) > self.angle_to_goal_max:
                # Simulate a point looking forward in the current baselink frame, and determine how much we should rotate based on a rate
                # if it is already too close, just point to the goal itself
                looking_forward_virtual_point = np.array(
                    [self.max_obstacle_distance, 0])
                direction_vector = goal_baselink_frame / \
                    np.linalg.norm(self.max_obstacle_distance) * \
                    self.max_obstacle_distance - looking_forward_virtual_point
                rate_change = 0.5 if np.linalg.norm(
                    direction_vector) > 3 else 1.0
                guided_point_baselink_frame = looking_forward_virtual_point + \
                    rate_change*direction_vector

                if time() - self.last_command_time > self.command_sending_interval:
                    self.sendGuidedPointGlobalFrame(
                        guided_point_baselink_frame=guided_point_baselink_frame)
                    self.last_command_time = time()
            else:
                # If angle is already low, return to AUTO mode
                if time() - self.last_command_time > self.command_sending_interval and self.current_state.mode != "AUTO":
                    self.setFlightMode(mode="AUTO")
                    self.last_command_time = time()

    # endregion


if __name__ == "__main__":
    try:
        obst_avoid_node = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass
