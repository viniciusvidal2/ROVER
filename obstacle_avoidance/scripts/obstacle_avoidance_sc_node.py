#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State, GlobalPositionTarget, WaypointList, HomePosition
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray
import numpy as np
from time import time
import utm
import logging
import os
from log_debug import *
from collision_lib import *


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node("obstacle_avoidance_node", anonymous=False)

        # User arguments
        self.guided_point_sending_interval = rospy.get_param(
            '~sendingT', 2)  # [s]
        self.max_obstacle_distance = rospy.get_param('~maxDist', 3)  # [m]

        # Control the node execution incoming messages
        self.last_input_scan_message_time = time()
        # Control the time when we last sent a guided point
        self.last_guided_point_time = time()
        # Variables
        self.current_yaw = 0.0  # [RAD]
        self.current_location = None  # GPS data
        self.current_state = State()  # vehicle driving mode
        self.debug_mode = True  # debug mode to print more information
        self.home_waypoint = None  # home waypoint data, contains home lat and lon
        self.waypoints_list = None  # list of waypoints in the autonomous mission
        self.current_target = None  # target waypoint data in AUTO mode
        self.utm_zone_number = None  # UTM zone number
        self.utm_zone_letter = None  # UTM zone letter
        self.vehicle_width = 0.4  # [m]

        # If the image and logging folders are not created, make sure we create it
        if self.debug_mode:
            self.debug_folder = os.path.join(
                os.getenv("HOME"), "obstacle_avoidance_debug")
            if not os.path.exists(os.path.join(self.debug_folder, "logs")):
                os.makedirs(os.path.join(self.debug_folder, "logs"))
            # Logging setup
            logging.basicConfig(
                filename=os.path.join(
                    self.debug_folder, f"logs/run_{getCurrentTimeAsString()}.log"),
                level=logging.DEBUG,
                format='%(asctime)s - %(levelname)s - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )

        # Subscribers to mavros and laserscan messages
        rospy.Subscriber("/livox/scan", LaserScan,
                         self.laserScanCallback, queue_size=1)
        rospy.Subscriber("/mavros/setpoint_raw/target_global",
                         GlobalPositionTarget, self.targetPointCallback, queue_size=1)
        rospy.Subscriber("/mavros/state", State,
                         self.stateCallback, queue_size=1)
        rospy.Subscriber("/mavros/global_position/global",
                         NavSatFix, self.gpsCallback, queue_size=1)
        rospy.Subscriber("/mavros/global_position/compass_hdg",
                         Float64, self.compassCallback, queue_size=1)
        rospy.Subscriber("/mavros/mission/waypoints",
                         WaypointList, self.missionWaypointsCallback, queue_size=10)
        rospy.Subscriber("/mavros/home_position/home",
                         HomePosition, self.homePositionCallback, queue_size=1)

        # If no message for some reason, resume original state with a frequency based callback
        self.timer_cb = rospy.Timer(rospy.Duration(
            1.0), self.travelStateCheckCallback)

        # Publishers
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
        self.obstacles_pub = rospy.Publisher(
            '/obstacle_avoidance/obstacles', MarkerArray, queue_size=1)
        self.goal_guided_point_pub = rospy.Publisher(
            '/obstacle_avoidance/goal_guided_point', MarkerArray, queue_size=1)
        self.robot_path_area_pub = rospy.Publisher(
            '/obstacle_avoidance/robot_path_area', Marker, queue_size=1)

        # Services
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo("Obstacle avoidance node initialized.")
        logging.info("Obstacle avoidance node initialized.")
        rospy.spin()

    ############################################################################
    # LOGGING
    ############################################################################

    def logCallbackLoop(self, obstacles_baselink_frame, goal_baselink_frame, guided_point_baselink_frame):
        # Log file
        logging.debug(f"Information for this loop: {time()}")
        logging.debug(f"Current state: {self.current_state.mode}")
        logging.debug(
            f"Current location: lat {self.current_location.latitude} lon {self.current_location.longitude}")
        logging.debug(f"Current yaw: {self.current_yaw*180.0/np.pi} degrees")
        logging.debug(
            f"Original target: lat {self.current_target.latitude} lon {self.current_target.longitude}")
        logging.debug(
            f"Goal direction in baselink frame: {goal_baselink_frame}")
        logging.debug(
            f"Guided point in baselink frame: {guided_point_baselink_frame}")
        for i, obstacle in enumerate(obstacles_baselink_frame):
            logging.debug(f"Obstacle {i} in baselink frame: {obstacle}")
        # Printing
        rospy.loginfo(
            "=================================================================")
        rospy.loginfo(f"Information for this loop: {time()}")
        rospy.loginfo(f"Current state: {self.current_state.mode}")
        rospy.loginfo(
            f"Current location: lat {self.current_location.latitude} lon {self.current_location.longitude}")
        rospy.loginfo(f"Current yaw: {self.current_yaw*180.0/np.pi} degrees")
        rospy.loginfo(
            f"Original target: lat {self.current_target.latitude} lon {self.current_target.longitude}")
        rospy.loginfo(
            f"Goal direction in baselink frame: {goal_baselink_frame}")
        rospy.loginfo(
            f"Guided point in baselink frame: {guided_point_baselink_frame}")
        x_target_baselink, y_target_baselink = self.worldToBaselink(
            target_lat=self.current_target.latitude, target_lon=self.current_target.longitude)
        rospy.loginfo(
            f"Target point heard from board in baselink frame: {x_target_baselink} x, {y_target_baselink} y")
        for i, obstacle in enumerate(obstacles_baselink_frame):
            rospy.loginfo(f"Obstacle {i} in baselink frame: {obstacle}")
        rospy.loginfo("Waypoints in baselink frame:")
        for i, waypoint in enumerate(self.waypoints_list):
            rospy.loginfo(
                f"Waypoint {i} in baselink frame: {self.worldToBaselink(target_lat=waypoint.x_lat, target_lon=waypoint.y_long)}")
        rospy.loginfo(
            "=================================================================")

    ############################################################################
    # FRAME CONVERTION METHODS
    ############################################################################

    def latLonToUtm(self, lat, lon):
        utm_e, utm_n, self.utm_zone_number, self.utm_zone_letter = utm.from_latlon(
            lat, lon)

        return utm_e, utm_n

    def utmToLatLon(self, utm_e, utm_n):
        lat, lon = utm.to_latlon(
            utm_e, utm_n, self.utm_zone_number, self.utm_zone_letter)

        return lat, lon

    def laserScanToXY(self, range, angle):
        x_baselink = range*np.cos(angle)
        y_baselink = range*np.sin(angle)
        if self.debug_mode:
            rospy.logwarn(
                f"Convertion using r={range} and angle={180/np.pi*angle}: x={x_baselink} y={y_baselink}")

        return x_baselink, y_baselink

    def baselinkToWorld(self, x_baselink, y_baselink):
        # Get current location from latlon to UTM coordinates
        utm_east, utm_north = self.latLonToUtm(
            lat=self.current_location.latitude, lon=self.current_location.longitude)
        # Create rotation from baselink to world based on the current yaw and apply
        world_angle_baselink = np.pi/2 - self.current_yaw
        ca = np.cos(world_angle_baselink)
        sa = np.sin(world_angle_baselink)
        world_R_baselink = np.array([[ca, -sa], [sa, ca]])
        d_utm = world_R_baselink @ np.array([x_baselink, y_baselink])
        # Add to the current location in UTM
        utm_output = np.array([utm_east, utm_north]) + d_utm

        return self.utmToLatLon(utm_e=utm_output[0], utm_n=utm_output[1])

    def worldToBaselink(self, target_lat, target_lon):
        # Get current location from latlon to UTM coordinates
        utm_east, utm_north = self.latLonToUtm(
            lat=self.current_location.latitude, lon=self.current_location.longitude)
        # Get the target location from latlon to UTM coordinates
        utm_target_east, utm_target_north = self.latLonToUtm(
            lat=target_lat, lon=target_lon)
        # Calculate the offset from the current location to the target location in UTM frame
        d_utm = np.array([utm_target_east, utm_target_north]
                         ) - np.array([utm_east, utm_north])
        # Create rotation from world to baselink based on the current yaw and apply
        baselink_angle_world = self.current_yaw - np.pi/2
        ca = np.cos(baselink_angle_world)
        sa = np.sin(baselink_angle_world)
        baselink_R_world = np.array([[ca, -sa], [sa, ca]])
        target_baselink_frame = baselink_R_world @ d_utm

        return target_baselink_frame[0], target_baselink_frame[1]

    ############################################################################
    # SENSOR CALLBACKS
    ############################################################################
    def targetPointCallback(self, data):
        # If we are in AUTO mode, we need to grab the next waypoint in the mission, if we do have a mission
        if self.waypoints_list:
            for waypoint in self.waypoints_list:
                if waypoint.is_current:
                    self.current_target = GlobalPositionTarget()
                    self.current_target.latitude = waypoint.x_lat
                    self.current_target.longitude = waypoint.y_long
                    self.current_target.altitude = waypoint.z_alt
                    if self.debug_mode:
                        rospy.loginfo(
                            f"Target point set to {self.current_target.latitude}, {self.current_target.longitude}.")
        if self.debug_mode:
            x_target_baselink, y_target_baselink = self.worldToBaselink(
                target_lat=self.current_target.latitude, target_lon=self.current_target.longitude)
            rospy.logwarn(
                f"Target in baselink frame: {x_target_baselink} x, {y_target_baselink} y.")
            rospy.loginfo(
                f"This is the current state: {self.current_state.mode}")

    def stateCallback(self, state):
        self.current_state = state

    def gpsCallback(self, data):
        self.current_location = data

    def compassCallback(self, data):
        # Convert compass heading from degrees to radians
        self.current_yaw = np.radians(data.data)  # [RAD]

    def missionWaypointsCallback(self, data):
        # Get the list of waypoints in the mission
        self.waypoints_list = data.waypoints
        # If the last waypoint coordinates are 0, that means it is a return to launch waypoint, so we add the home waypoint values to this waypoint
        if self.waypoints_list[-1].x_lat == 0 and self.waypoints_list[-1].y_long == 0 and self.home_waypoint:
            self.waypoints_list[-1].x_lat = self.home_waypoint.geo.latitude
            self.waypoints_list[-1].y_long = self.home_waypoint.geo.longitude
        # Get the current target waypoint if we are in a mission
        if self.current_state.mode == "AUTO":
            for waypoint in self.waypoints_list:
                if waypoint.is_current:
                    self.current_target = GlobalPositionTarget()
                    self.current_target.latitude = waypoint.x_lat
                    self.current_target.longitude = waypoint.y_long
                    self.current_target.altitude = waypoint.z_alt
                    break

    def homePositionCallback(self, data):
        # Convert the data to UTM just to store in the class our zone number and letter
        _, _ = self.latLonToUtm(lat=data.geo.latitude, lon=data.geo.longitude)
        # Set the home point so we know what to do if we are returning to launch
        if not self.home_waypoint:
            self.home_waypoint = data
        else:
            if self.home_waypoint.geo.latitude != data.geo.latitude or self.home_waypoint.geo.longitude != data.geo.longitude:
                self.home_waypoint = data
        if self.debug_mode and self.home_waypoint:
            rospy.loginfo(
                f"Home waypoint set to {self.home_waypoint.geo.latitude}, {self.home_waypoint.geo.longitude}")

    def travelStateCheckCallback(self, event):
        # Check if we are some time with no input data, and if so get back to AUTO mode mission
        if time() - self.last_input_scan_message_time > 4:
            rospy.logwarn(
                "No data was received in the last 4 seconds. Returning to original state ...")
            self.resumeOriginalState()

    ############################################################################
    # CONTROL FUNCTIONS
    ############################################################################
    def setFlightMode(self, mode):
        try:
            response = self.set_mode_service(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Changing navigation mode to {mode}...")
                while self.current_state.mode != mode:
                    response = self.set_mode_service(custom_mode=mode)
                    rospy.loginfo(
                        "Waiting for mode change request confirmation ...")
                    rospy.sleep(0.1)
                rospy.loginfo(f"Mode {mode} activated.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to change navigation mode to {mode}: {e}")

    def resumeOriginalState(self):
        # Try to get back to AUTO mode after obstacle is avoided
        # If AUTO, resume mission where it left of
        # If any errors, get back to manual mode. Of course, just stay in MANUAL mode if we are already there
        if self.current_state.mode == "MANUAL":
            return

        if self.current_state.mode == "GUIDED":
            try:
                self.setFlightMode("AUTO")
                rospy.loginfo(f"Resuming mission in AUTO mode.")
            except rospy.ServiceException as e:
                self.setFlightMode("MANUAL")
                rospy.logerr(
                    f"Failed to resume original state, setting MANUAL mode: {e}")

    def createAngleTestSequence(self, goal_angle, angle_step, full_test_range):
        # Lets have a list of angles to test, starting from the goal angle and going in both directions
        # in a breadth first search manner
        angles = list()  # [RAD]
        angles.append(np.radians(goal_angle))
        for i in range(angle_step, full_test_range, angle_step):
            angles.append(np.radians(goal_angle + i))
            angles.append(np.radians(goal_angle - i))

        return angles

    def calculateBestTrajectoryGuidedPoint(self, angle_tests, goal_distance, obstacles_baselink_frame_xy):
        # Check if the path is clear for each angle, and return the first one that is clear
        path_found = False
        chosen_angle = angle_tests[0]
        for angle in angle_tests:
            if testPointsInRotatedRectangle(obstacles_baselink_frame_xy, length=goal_distance, width=self.vehicle_width, angle=angle):
                path_found = True
                chosen_angle = angle
                break
        if self.debug_mode:
            rospy.loginfo(
                f"Chosen angle for the trajectory: {180/np.pi*chosen_angle} degrees")
            rospy.loginfo(
                f"Distance from center angle: {180/np.pi*(chosen_angle - angle_tests[0])}")
            rospy.loginfo(f"Path found: {path_found}")

        # If we found a path, lets calculate the guided point based on the chosen angle using the goal distance
        if path_found:
            guided_point_x = goal_distance * np.cos(chosen_angle)
            guided_point_y = goal_distance * np.sin(chosen_angle)
            return [guided_point_x, guided_point_y], 180/np.pi*abs(chosen_angle - angle_tests[0])
        else:
            rospy.logerr("No path was found to avoid obstacles!")
            return 0, 0, 180/np.pi*abs(angle_tests[-1] - angle_tests[0])

    ############################################################################
    # MAIN CONTROL LOOP CALLBACK
    ############################################################################
    def laserScanCallback(self, scan):
        """
        We use distance sensor to calculate the potential fields forces and find the waypoint we should travel to.
        We work with two main frames: baselink and world
        Baselink: X forward, Y left of the vehicle. 0~360 degrees, counter-clockwise, 0 is in the back
        World: latlon, so X (lat) points up and Y (lon) points to the right. 0~360 degrees, clockwise, 0 is in positive X 
        """
        # Track message receiving so we can resume mission in case there is no messages arriving
        self.last_input_scan_message_time = time()

        # Avoiding the callback if the conditions are not met
        if not scan.ranges or self.current_state.mode == "MANUAL" or not self.current_target \
                or not self.current_location or not self.utm_zone_letter or not self.utm_zone_number:
            return

        # Make sure the scan values are valid before doing any math
        valid_ranges = np.array(scan.ranges)
        valid_ranges[valid_ranges == 0] = 1e6
        closest_obstacle_distance = np.min(valid_ranges)
        # If any point is close enough, process the avoidance behavior
        if closest_obstacle_distance < self.max_obstacle_distance:
            if self.debug_mode:
                rospy.logwarn(
                    f"Obstacle detected in less than {self.max_obstacle_distance}m!")

            # Lets only proceed if there is enough time since we last sent a guided point to the vehicle
            if time() - self.last_guided_point_time < self.guided_point_sending_interval:
                return
            self.last_guided_point_time = time()

            # In case there is a target waypoint, we can calculate the avoidance
            if self.current_target:
                # Grab the goal direction in baselink frame
                goal_baselink_frame = self.worldToBaselink(
                    target_lat=self.current_target.latitude, target_lon=self.current_target.longitude)
                goal_angle_baselink_frame = np.arctan2(
                    goal_baselink_frame[1], goal_baselink_frame[0])
                goal_distance = np.linalg.norm(goal_baselink_frame)

                # Calculate the angles we will test to create the best trajectory
                angle_tests = self.createAngleTestSequence(
                    goal_angle=180/np.pi*goal_angle_baselink_frame, angle_step=5, full_test_range=90)
                if self.debug_mode:
                    rospy.loginfo(
                        f"Goal direction in baselink frame: {goal_baselink_frame}")
                    rospy.loginfo(f"Goal distance: {goal_distance} m")
                    rospy.loginfo(
                        f"Goal angle in baselink frame: {180/np.pi*goal_angle_baselink_frame} degrees")

                # Isolate the readings that return the obstacles - obstacles are in pairs of (range, angle) in baselink frame
                obstacles_baselink_frame = [[r, i * scan.angle_increment + scan.angle_min]
                                            for i, r in enumerate(valid_ranges) if r < self.max_obstacle_distance]
                obstacles_baselink_frame_xy = [self.laserScanToXY(
                    range=r, angle=a) for r, a in obstacles_baselink_frame]
                if self.debug_mode:
                    self.obstacles_pub.publish(
                        createObstaclesDebugMarkerArray(obstacles_baselink_frame_xy))

                # Get the best trajectory from the shapes we are observing, starting from the goal point angle
                guided_point_baselink_frame, guided_to_goal_angle = self.calculateBestTrajectoryGuidedPoint(
                    angle_tests=angle_tests, goal_distance=goal_distance, obstacles_baselink_frame_xy=obstacles_baselink_frame_xy)

                # Start avoiding and set the GUIDED mode to send commands
                # If we should in fact just keep to the goal, meaning no obstacles are in the way, we should go back to AUTO mode
                if self.current_state.mode == "AUTO" and guided_to_goal_angle > 5:
                    self.setFlightMode("GUIDED")
                    if self.debug_mode:
                        rospy.logwarn(
                            "Avoiding obstacles by setting GUIDED mode ...")
                elif self.current_state.mode == "GUIDED" and guided_to_goal_angle < 5:
                    self.setFlightMode("AUTO")
                    if self.debug_mode:
                        rospy.logwarn(
                            "No obstacles in the way, resuming AUTO mode ...")

                # Convert the travel point to world frame
                guided_point_world_frame_lat, guided_point_world_frame_lon = self.baselinkToWorld(
                    x_baselink=guided_point_baselink_frame[0], y_baselink=guided_point_baselink_frame[1])

                # Send the new point to the vehicle
                guided_point_world_frame_msg = GlobalPositionTarget()
                guided_point_world_frame_msg.latitude = guided_point_world_frame_lat
                guided_point_world_frame_msg.longitude = guided_point_world_frame_lon
                guided_point_world_frame_msg.altitude = self.current_location.altitude
                self.setpoint_pub.publish(guided_point_world_frame_msg)

                # Publish goal and target points for debug purposes
                if self.debug_mode:
                    self.goal_guided_point_pub.publish(createGoalGuidedPointDebugMarkerArray(
                        goal=goal_baselink_frame, guided_point=guided_point_baselink_frame))
                    self.robot_path_area_pub.publish(createRobotPathAreaMarker(
                        height=goal_distance, width=self.vehicle_width, angle=goal_angle_baselink_frame))
                    # Log the complete loop information
                    self.logCallbackLoop(
                        obstacles_baselink_frame, goal_baselink_frame, guided_point_baselink_frame)

                if self.debug_mode:
                    end_time = time()
                    rospy.loginfo(
                        f"Time to process avoidance: {1000*(end_time - self.last_input_scan_message_time)} milliseconds.")

        elif self.current_state.mode == "GUIDED":
            if self.debug_mode:
                rospy.logwarn(
                    "No obstacle observed nearby, resuming original mission ...")
            self.resumeOriginalState()


if __name__ == "__main__":
    try:
        obst_avoid_node = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass
