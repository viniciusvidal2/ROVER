#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State, GlobalPositionTarget, WaypointList, HomePosition, PositionTarget
from mavros_msgs.srv import SetMode, WaypointSetCurrent
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
from time import time
from log_debug import *
from collision_lib import *
from frame_convertions import *


class ObstacleAvoidance:
    ############################################################################
    # region CONSTRUCTOR
    ############################################################################
    def __init__(self):
        rospy.init_node("obstacle_avoidance_node", anonymous=False)

        # User arguments
        self.guided_point_sending_interval = rospy.get_param(
            '~sending_t', 3)  # [s]
        self.max_obstacle_distance = rospy.get_param('~max_dist', 3)  # [m]

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
        self.current_waypoint_index = -1  # Autonomous mission waypoint we are tracking
        self.current_target = None  # target waypoint data in AUTO mode
        self.vehicle_width = 1.5  # [m]
        self.critical_zone_radius = 1.0  # [m]

        # Subscribers to mavros and laserscan messages
        rospy.Subscriber("/livox/scan", LaserScan,
                         self.laserScanCallback, queue_size=1)
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
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_current_wp_srv = rospy.ServiceProxy(
            '/mavros/mission/set_current', WaypointSetCurrent)

        rospy.logwarn("Obstacle avoidance node initialized.")
        rospy.spin()

    # endregion
    ############################################################################
    # region SENSOR CALLBACKS
    ############################################################################

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
        if len(self.waypoints_list) == 0:
            return
        # If the last waypoint coordinates are 0, that means it is a return to launch waypoint, so we add the home waypoint values to this waypoint
        if self.waypoints_list[-1].x_lat == 0 and self.waypoints_list[-1].y_long == 0 and self.home_waypoint:
            self.waypoints_list[-1].x_lat = self.home_waypoint.geo.latitude
            self.waypoints_list[-1].y_long = self.home_waypoint.geo.longitude
        # Get the current target waypoint if we are in a mission
        if self.current_state.mode == "AUTO":
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

    def homePositionCallback(self, data):
        # Set the home point so we know what to do if we are returning to launch
        if not self.home_waypoint:
            self.home_waypoint = data
        else:
            if self.home_waypoint.geo.latitude != data.geo.latitude or self.home_waypoint.geo.longitude != data.geo.longitude:
                self.home_waypoint = data
        if self.debug_mode and self.home_waypoint:
            rospy.logwarn(
                f"Home waypoint set to {self.home_waypoint.geo.latitude}, {self.home_waypoint.geo.longitude}")

    def travelStateCheckCallback(self, event):
        # Check if we are some time with no input data, and if so get back to AUTO mode mission
        if time() - self.last_input_scan_message_time > 4:
            rospy.logwarn(
                "No data was received in the last 4 seconds. Returning to original state ...")
            self.resumeOriginalState()

    # endregion
    ############################################################################
    # region CONTROL FUNCTIONS
    ############################################################################
    def setFlightMode(self, mode):
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

    def resumeOriginalState(self):
        # Try to get back to AUTO mode after obstacle is avoided
        # If AUTO, resume mission where it left of
        # If any errors, get back to manual mode. Of course, just stay in MANUAL mode if we are already there
        if self.current_state.mode == "MANUAL":
            return

        if self.current_state.mode == "GUIDED":
            try:
                self.setFlightMode("AUTO")
                rospy.logwarn(f"Resuming mission in AUTO mode.")
            except rospy.ServiceException as e:
                self.setFlightMode("MANUAL")
                rospy.logerr(
                    f"Failed to resume original state, setting MANUAL mode: {e}")

    def setCurrentTargetToPreviousWaypoint(self):
        # Lets try to go back to the previous point
        rospy.logerr(
            "No path was found to avoid obstacles, going back to previous waypoint in mission, if any!")
        previous_waypoint_index = self.current_waypoint_index - 1
        if self.current_waypoint_index == 2:
            previous_waypoint_index = 0
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

    def sendGuidedPointBodyFrame(self, guided_point_baselink_frame):
        # Calculate guided point angle:
        angle = -np.arctan2(guided_point_baselink_frame[1], guided_point_baselink_frame[0])
        
        setpoint = PositionTarget()
        setpoint.header = Header()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "base_link"  # This represents the body frame
        # Set the coordinate frame to FRAME_BODY_NED (1) for body frame
        setpoint.coordinate_frame = PositionTarget.FRAME_BODY_NED
        # Set position (changing y and angle to cope with XY -> NE)
        setpoint.position.x = guided_point_baselink_frame[0]
        setpoint.position.y = -guided_point_baselink_frame[1]
        setpoint.position.z = 0.0
        setpoint.velocity.x = 0.0
        setpoint.velocity.y = 0.0
        setpoint.velocity.z = 0.0
        setpoint.acceleration_or_force.x = 0.0
        setpoint.acceleration_or_force.y = 0.0
        setpoint.acceleration_or_force.z = 0.0
        setpoint.yaw = angle
        setpoint.yaw_rate = 0.0
        # Specify which fields are valid
        setpoint.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ \
            | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
            | PositionTarget.IGNORE_YAW_RATE

        self.setpoint_local_pub.publish(setpoint)

    def sendGuidedPointGlobalFrame(self, guided_point_baselink_frame):
        # Convert the travel point to world frame
        guided_point_world_frame_lat, guided_point_world_frame_lon = baselinkToWorld(
            xy_baselink=np.asarray(guided_point_baselink_frame),
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
    # region MAIN CONTROL LOOP CALLBACK
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
        if not scan.ranges or self.current_state.mode == "MANUAL" or not self.current_target or not self.current_location or not self.home_waypoint:
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
            # If inside the critical zone, always act
            if time() - self.last_guided_point_time < self.guided_point_sending_interval:
                return
            self.last_guided_point_time = time()

            # In case there is a target waypoint, we can calculate the avoidance
            if self.current_target:
                # Grab the goal direction in baselink frame
                goal_baselink_frame = worldToBaselink(
                    target_lat=self.current_target.latitude, target_lon=self.current_target.longitude,
                    current_location=self.current_location, current_yaw=self.current_yaw)
                goal_angle_baselink_frame = np.arctan2(
                    goal_baselink_frame[1], goal_baselink_frame[0])
                goal_distance = np.linalg.norm(goal_baselink_frame)
                # If we are closer to goal, just continue to the goal
                if goal_distance < closest_obstacle_distance:
                    self.setFlightMode("AUTO")
                    return

                # Checking if we are in a critical zone, so that we can act in the body frame starting from the 0 angle
                critical_zone = closest_obstacle_distance < self.critical_zone_radius

                # Calculate the angles we will test to create the best trajectory, minding the critical zone
                angle_tests = createAngleTestSequence(
                    goal_angle=np.degrees(goal_angle_baselink_frame), angle_step=5, full_test_range=90) if not critical_zone \
                    else createAngleTestSequence(goal_angle=0, angle_step=5, full_test_range=90)
                if self.debug_mode:
                    rospy.logwarn(
                        f"Goal direction in baselink frame: {goal_baselink_frame}")
                    rospy.logwarn(f"Goal distance: {goal_distance} m")
                    rospy.logwarn(
                        f"Goal angle in baselink frame: {np.degrees(goal_angle_baselink_frame)} degrees")

                # Isolate the readings that return the obstacles - obstacles are in pairs of (range, angle) in baselink frame
                obstacles_baselink_frame = [[r, i * scan.angle_increment + scan.angle_min]
                                            for i, r in enumerate(valid_ranges) if r < self.max_obstacle_distance]
                obstacles_baselink_frame_xy = [laserScanToXY(
                    range=r, angle=a) for r, a in obstacles_baselink_frame]
                if self.debug_mode:
                    self.obstacles_pub.publish(
                        createObstaclesDebugMarkerArray(obstacles_baselink_frame_xy))

                # Get the best trajectory from the shapes we are observing, starting from the goal point angle
                guided_point_baselink_frame, guided_to_goal_angle = calculateBestTrajectoryGuidedPoint(
                    angle_tests=angle_tests, goal_distance=goal_distance, obstacles_baselink_frame_xy=obstacles_baselink_frame_xy,
                    vehicle_width=self.vehicle_width)

                # If we have an issue finding this trajectory, we should go back to the previous waypoint
                if np.linalg.norm(guided_point_baselink_frame) == 0:
                    rospy.logerr(
                        "No path was found to avoid obstacles, going back to previous waypoint in mission!")
                    self.setCurrentTargetToPreviousWaypoint()
                    self.setFlightMode("AUTO")
                    return

                # Start avoiding and set the GUIDED mode to send commands
                # If we should in fact just keep to the goal, meaning no obstacles are in the way, we should go back to AUTO mode
                if self.current_state.mode == "AUTO" and guided_to_goal_angle > 5:
                    self.setFlightMode("GUIDED")
                elif self.current_state.mode == "GUIDED" and guided_to_goal_angle < 5:
                    # Check if there is enough FOV to the goal before changing to AUTO mode, which means we have
                    # now quite left the obstacle behind
                    if checkSafeFOV(obstacles_baselink_frame, goal_baselink_frame):
                        self.setFlightMode("AUTO")
                        return

                # If it is a critical zone, we should act in the body frame, shortening the goal distance
                if critical_zone:
                    critical_point_distance = np.max(
                        [closest_obstacle_distance, 2])
                    guided_point_baselink_frame = guided_point_baselink_frame / \
                        np.linalg.norm(guided_point_baselink_frame) * \
                        critical_point_distance
                    self.sendGuidedPointBodyFrame(guided_point_baselink_frame)
                else:
                    self.sendGuidedPointGlobalFrame(
                        guided_point_baselink_frame)

                # Publish goal and target points for debug purposes
                if self.debug_mode:
                    self.goal_guided_point_pub.publish(createGoalGuidedPointDebugMarkerArray(
                        goal=goal_baselink_frame, guided_point=guided_point_baselink_frame))
                    guided_point_angle = np.arctan2(
                        guided_point_baselink_frame[1], guided_point_baselink_frame[0])
                    self.robot_path_area_pub.publish(createRobotPathAreaMarker(
                        height=goal_distance, width=self.vehicle_width, angle=guided_point_angle))
                    # Log the complete loop information
                    logCallbackLoop(
                        obstacles_baselink_frame=obstacles_baselink_frame_xy, goal_baselink_frame=goal_baselink_frame,
                        guided_point_baselink_frame=guided_point_baselink_frame,
                        current_state=self.current_state, current_location=self.current_location,
                        current_yaw=self.current_yaw, current_target=self.current_target,
                        current_waypoint_index=self.current_waypoint_index, target_baselink=goal_baselink_frame)
                    end_time = time()
                    rospy.logwarn(
                        f"Time to process avoidance: {1000*(end_time - self.last_input_scan_message_time)} milliseconds.")

        elif self.current_state.mode == "GUIDED":
            self.resumeOriginalState()

    # endregion


if __name__ == "__main__":
    try:
        obst_avoid_node = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass
