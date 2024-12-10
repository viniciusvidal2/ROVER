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
from collision_lib import *
from frame_convertions import *
from log_debug import *
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


class ObstacleAvoidance:
    ############################################################################
    # region CONSTRUCTOR
    ############################################################################
    def __init__(self) -> None:
        rospy.init_node("obstacle_avoidance_node", anonymous=False)

        # User arguments
        self.command_sending_interval = rospy.get_param(
            '~sending_t', 0.15)  # [s] 
        self.max_obstacle_distance = rospy.get_param('~max_dist', 3)  # [m]
        self.corridor_width = rospy.get_param('~corridor_width', 5)  # [m]
        self.turn_rate = rospy.get_param('~turn_rate', 30) # [degrees]

        # Control the time when we last sent a guided point
        self.last_command_time = time()
        # Variables
        self.current_yaw = 0.0  # [RAD]
        self.current_location = None  # GPS data
        self.current_state = State()  # vehicle driving mode
        # self.debug_mode = False  # debug mode to print more information
        self.debug_mode = True
        self.home_waypoint = None  # home waypoint data, contains home lat and lon
        self.waypoints_list = None  # list of waypoints in the autonomous mission
        self.current_waypoint_index = -1  # autonomous mission waypoint we are tracking
        self.current_target = None  # target waypoint data in AUTO mode
        self.angle_to_goal_max = 10  # [degrees]
        self.start_search_side = 'l'  # side to start looking for available paths, l or r
        self.previous_guided_point_angle = None  # [RAD]
        self.previous_current_turn_angle_threshold = 45.0*np.pi/180.0  # [RAD]
        self.low_pass_guided_point_angle = self.turn_rate*np.pi/180.0  # [RAD]

        # Parameters from obstacle avoidance algorithm
        # self.dt = 0.17 # Time step [s] 
        self.closest_obstacle_distance = 0
        self.x = 0 # Actual x position from odometry
        self.y = 0 # Actual y position from odometry
        self.theta = 0 # Actual orientation from odometry
        self.v = 0 # Actual linear velocity from odometry
        self.w = 0 # Actual angular velocity from odometry
        self.max_v = 2.55 # Maximum linear velocity 0.5
        self.max_w = np.pi/2 # Maximum angular velocity
        self.best_v = None # Best linear velocity
        self.best_w = None # Best angular velocity
        # self.max_w = np.pi/3
        self.min_v = 0.0 # Minimum linear velocity
        self.max_acc_v = 0.6 # Maximum linear acceleration 0.5
        self.max_acc_w = np.pi/2 # Maximum angular acceleration
        self.safety_distance_to_end = 3.0 # [meters] 
        self.safety_distance_to_start = 5.0 # [meters] 3.0, 4.0
        self.valid_ranges = None
        self.next_waypoint_dist = 3.0 # [meters]

        self.v_reso = 30 # Linear velocity resolution
        self.w_reso = 20 # Angular velocity resolution

        # self.alpha = 2.0 # Robot alignment to the objective
        # self.beta = 0.2 # Distance to the obstacle
        # self.gamma = 0.2 # Foward speed
        
        # self.alpha = 0.5 # Robot alignment to the objective
        # self.beta = 2.0 # Distance to the obstacle
        # self.gamma = 0.2 # Foward speed

        self.alpha = 1.0 # Robot alignment to the objective
        self.beta = 2.0 # Distance to the obstacle
        self.gamma = 0.2 # Foward speed

        self.angle_min = -2.268889904022217
        self.angle_max = 2.268899917602539
        self.goal_angle = 0

        # Subscribers to mavros and laserscan messages
        '''rospy.Subscriber("/rover_1/mavros/lidar", LaserScan,
                         self.laserScanCallback, queue_size=1)
        rospy.Subscriber("/rover_1/mavros/state", State,
                         self.stateCallback, queue_size=1)
        rospy.Subscriber("/rover_1/mavros/global_position/global",
                         NavSatFix, self.gpsCallback, queue_size=1)
        rospy.Subscriber("/rover_1/mavros/global_position/compass_hdg",
                         Float64, self.compassCallback, queue_size=1)
        rospy.Subscriber("/rover_1/mavros/setpoint_raw/target_global",
                         GlobalPositionTarget, self.currentTargetCallback, queue_size=1)
        rospy.Subscriber("/rover_1/mavros/mission/waypoints",
                         WaypointList, self.missionWaypointsCallback, queue_size=1)
        rospy.Subscriber("/rover_1/mavros/home_position/home",
                         HomePosition, self.homePositionCallback, queue_size=1)
        rospy.Subscriber('/rover_1/mavros/local_position/odom', 
                                         Odometry, self.odom_callback)'''
        
        rospy.Subscriber("/mavros/lidar", LaserScan,
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
        rospy.Subscriber('/mavros/local_position/odom', 
                                         Odometry, self.odom_callback)

        # Publishers
        '''self.setpoint_global_pub = rospy.Publisher(
            '/rover_1/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
        self.setpoint_local_pub = rospy.Publisher(
            '/rover_1/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.obstacles_pub = rospy.Publisher(
            '/rover_1/obstacle_avoidance/obstacles', MarkerArray, queue_size=1)
        self.goal_guided_point_pub = rospy.Publisher(
            '/rover_1/obstacle_avoidance/goal_guided_point', MarkerArray, queue_size=1)
        self.robot_path_area_pub = rospy.Publisher(
            '/rover_1/obstacle_avoidance/robot_path_area', Marker, queue_size=1)'''
        
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
        '''rospy.wait_for_service('/rover_1/mavros/set_mode')
        rospy.wait_for_service('/rover_1/mavros/mission/set_current')
        rospy.wait_for_service('/rover_1/mavros/cmd/command')
        self.set_mode_service = rospy.ServiceProxy('/rover_1/mavros/set_mode', SetMode)
        self.set_current_wp_srv = rospy.ServiceProxy(
            '/rover_1/mavros/mission/set_current', WaypointSetCurrent)
        self.command_tol_srv = rospy.ServiceProxy(
            '/rover_1/mavros/cmd/command', CommandTOL)'''
        
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

    def odom_callback(self, msg: Odometry) -> None:
        """Receive odometry data and update robot position and orientation. 
        
        Args:
            msg (Odometry): Odometry message from mavros
        """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x = position.x # [meters]
        self.y = position.y # [meters]
        orientation_list = [orientation.x,orientation.y,orientation.z,orientation.w]
        # Converte quaternionic orientation representation into Euler angles (X,Y,Z)
        _,_, self.theta = euler_from_quaternion(orientation_list) # [rad]
        self.v = msg.twist.twist.linear.x # [m/s]
        self.w = msg.twist.twist.angular.z # [rad/s]

        current_time = msg.header.stamp.to_sec()

        # Calcula aceleração se houver uma velocidade anterior registrada
        if hasattr(self, 'previous_velocity') and hasattr(self, 'previous_time'):
            self.dt = current_time - self.previous_time
            if self.dt > 0:
                self.acceleration = (self.v - self.previous_velocity) / self.dt
                # rospy.loginfo(f"v: {self.v} m/s, w: {self.w} rad/s, acc: {self.acceleration} m/s²")
    
        # Armazena a velocidade e o tempo atuais para a próxima leitura
        self.previous_velocity = self.v
        self.previous_time = current_time

    def stateCallback(self, state: State) -> None:
        """Current vehicle driving state.

        Args:
            state (State): current vehicle driving state
        """
        self.current_state = state

    def gpsCallback(self, data: NavSatFix) -> None:
        """Get GPS data.

        Args:
            data (NavSatFix): gps data from mavros
        """
        self.current_location = data

    def compassCallback(self, data: Float64) -> None:
        """Compass data for orientation.

        Args:
            data (Float64): yaw angle, from 0~360 degrees, 0 to the north, increasing clockwise. 
        """
        self.current_yaw = np.radians(data.data)  # [RAD]

    def currentTargetCallback(self, data: GlobalPositionTarget) -> None:
        """Current target from mavros, usually the next item in the autonomous mission.

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
        """Mission currently set in the pixhawk board.

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

        # if self.debug_mode:
        #     rospy.logwarn(
        #         f"Received mission waypoints in mission waypoints callback: {len(self.waypoints_list)} waypoints.")
        #     for i, waypoint in enumerate(self.waypoints_list):
        #         rospy.logwarn(
        #             f"Mission waypoint {i+1}/{len(self.waypoints_list)}: {waypoint.x_lat}, {waypoint.y_long}, {waypoint.z_alt}")
        #         if waypoint.is_current:
        #             rospy.logwarn(
        #                 f"Current mission waypoint: {waypoint.x_lat}, {waypoint.y_long}, {waypoint.z_alt}")

        # If we are in GUIDED mode, no need to update the current target
        if self.current_state.mode == "GUIDED":           
            return

        # Get the current target waypoint if we are in a mission and not in GUIDED mode
        for i, waypoint in enumerate(self.waypoints_list):
            if waypoint.is_current:

                new_waypoint = waypoint
                self.current_waypoint_index = i

                self.current_target = GlobalPositionTarget()
                self.current_target.latitude = new_waypoint.x_lat
                self.current_target.longitude = new_waypoint.y_long
                self.current_target.altitude = new_waypoint.z_alt
                
                if self.debug_mode:
                    rospy.logwarn(
                        f"Target point set to {self.current_target.latitude}, {self.current_target.longitude} in mission callback.")

                break

    def homePositionCallback(self, data: HomePosition) -> None:
        """Last home waypoint set by the board, before or right at the mission start.

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
        """Set the flight mode we want to navigate with.

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
        """Tries to return to previous waypoint if no path to the next waypoint is observed.
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

    def advance_to_next_waypoint(self, index) -> None:
        """Advance to the next waypoint in the list."""

        if not self.current_waypoint_index:
            return

        # Check if we have reached the end of the waypoint list
        if index >= len(self.waypoints_list):
            rospy.logwarn("All waypoints completed.")
            self.current_target = None  # No more waypoints to follow
            return

        # New waypoint index
        next_waypoint_index = index

        try:
            response = self.set_current_wp_srv(next_waypoint_index)
            if response.success:
                rospy.logwarn(
                    f"Successfully set current waypoint to {next_waypoint_index}")
            else:
                rospy.logwarn(
                    f"Failed to set current waypoint to {next_waypoint_index}")
        except rospy.ServiceException as e:
            rospy.logerr(
                f"Waypoint set service call failed for index {next_waypoint_index}: {e}")
            
        # Set the next waypoint as the current target
        self.current_waypoint_index = next_waypoint_index
        self.current_target = GlobalPositionTarget()
        self.current_target.latitude = self.waypoints_list[next_waypoint_index].x_lat
        self.current_target.longitude = self.waypoints_list[next_waypoint_index].y_long
        self.current_target.altitude = self.waypoints_list[next_waypoint_index].z_alt    

        if self.debug_mode:
            rospy.loginfo(
                f"Target point set to {self.current_target.latitude}, {self.current_target.longitude} during obstacle avoidance.") 
            
    
    def find_safe_waypoint(self):
        """Find the closest waypoint to the current position using."""

        # Create a numpy array from the waypoints coordinates
        waypoints_array = np.array([(wp.x_lat, wp.y_long) for wp in self.waypoints_list])

        # Calculate the distances
        distances = np.linalg.norm(waypoints_array - np.array([self.current_location.latitude, self.current_location.longitude]), axis=1)

        # Find the index of the first waypoint that has a distance less than distance_to_start
        failed_indices = np.where(distances < self.safety_distance_to_start)[0]

        # If there are no failed waypoints, return None
        if failed_indices.size == 0:
            return None
        
        # Get the index of the last failed waypoint
        last_failed_index = failed_indices[-1]

        safe_index = last_failed_index + 1

        if last_failed_index + 1 >= len(self.waypoints_list):
            return None 

        return safe_index

    def sendGuidedPointLocalFrame(self, best_v, best_w) -> None:
        """Send a specified velocity.
        
        Args:
            best_v (float): best linear velocity.
            best_w (float): best angular velocity.
        """ 
        # Create local velocity message (PositionTarget)
        guided_point_local_frame_msg = PositionTarget()
        guided_point_local_frame_msg.header = Header()
        guided_point_local_frame_msg.header.stamp = rospy.Time.now()

        # Ignore position, acceleration and focus only on velocity
        guided_point_local_frame_msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # You can control YAW separately, if it's necessary
        )

        # Define frame coordenation (BODY_NED, for example)
        guided_point_local_frame_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED

        # Configurate desired velocities [m/s] and [rad/s]
        guided_point_local_frame_msg.velocity.x = best_v  # Velocity in x direction
        guided_point_local_frame_msg.velocity.y = 0.0     # No lateral movement
        guided_point_local_frame_msg.velocity.z = 0.0     # No vertical movement
        guided_point_local_frame_msg.yaw_rate = best_w    # Angular velocity in z direction

        # Publish message
        self.setpoint_local_pub.publish(guided_point_local_frame_msg)

    # endregion
    ############################################################################
    # region AVOIDANCE METHODS
    ############################################################################

    def dynamic_window(self):
        """Calculate dynamic window obtaining the possible velocities for the next interval of time. 
        
        Returns:
            min_v (float): minimal linear velocity,
            max_v (float): maximal linear velocity,
            min_w (float): minimal angular velocity,
            max_w (float): maximal angular velocity."""

        # Calculate dynamic window
        min_v = max(self.min_v, self.v - self.max_acc_v * self.dt) # The second part simulates the maximum deceleration
        max_v = min(self.max_v, self.v + self.max_acc_v * self.dt) # The second part simulates the maximum acceleration
        min_w = max(-self.max_w, self.w - self.max_acc_w * self.dt)
        max_w = min(self.max_w, self.w + self.max_acc_w * self.dt)
        
        return min_v, max_v, min_w, max_w

    def dynamic_window_safety_stop(self, dist_obst):
        """Calculate dynamic window to ensure that the robot can stop in time to avoid a collision.
        Based on dynamic restriction.
        
        Args:
            dist_obst (float): distance to the closest obstacle.
        Returns:
            safe_v_max (float): maximal linear velocity to stop in time,
            safe_w_max (float): maximal angular velocity to stop in time."""
        
        safe_v_max = np.sqrt(2 * dist_obst * self.max_acc_v)
        safe_w_max = np.sqrt(2 * dist_obst * self.max_acc_w)
        
        return safe_v_max, safe_w_max
    
    def get_fov_index_from_theta(self, theta):
        """
        Converte an angle (theta) in relation to the robot frame to the Lidar index.
        
        Args:
            theta: the angle in relation to the robot frame (in radians).
            
        Returns:
            The corresponding index in the Lidar field of view.
        """
        angle_min = self.angle_min # Assume that the Lidar has a 360 degree FOV (from -pi to pi)
        angle_max = self.angle_max
        num_readings = len(self.valid_ranges)

        # Converte the angle to the corresponding index in the Lidar
        fov_index = int((theta - angle_min) / (angle_max - angle_min) * num_readings)

        # Guarantees that the index is within the array limits
        fov_index = np.clip(fov_index, 0, num_readings - 1)
        
        return fov_index
    
    def objective_function(self, min_v, max_v, min_w, max_w):
        """Objective function to be maximazed.
        Args:
            min_v (float): minimal linear velocity,
            max_v (float): maximal linear velocity,
            min_w (float): minimal angular velocity,
            max_w (float): maximal angular velocity.
        Returns:
            best_v (float): best linear velocity,
            best_w (float): best angular velocity."""

        max_cost = -float('inf') # Initialize cost with negative infinity
        best_v, best_w = 0, 0  

        for v in np.linspace(min_v, max_v, num=self.v_reso): 
            for w in np.linspace(min_w, max_w, num=self.w_reso):

                # Preview the new robot orientation (theta) after applying w
                theta_next = self.theta + w * self.dt

                # Find the corresponding direction in the Lidar field of view
                fov_index = self.get_fov_index_from_theta(theta_next)

                # Obtain the distance to the obstacle in that direction
                dist_obst = self.valid_ranges[fov_index]

                # If no obstacle is on the curvature, this value is set to a large constant
                if np.isinf(dist_obst) or dist_obst > self.safety_distance_to_start:
                    dist_obst = 1000
                # if np.isinf(dist_obst) or dist_obst > self.safety_distance_to_start:
                #     dist_obst = 3.0
                
                else:
                    safe_v_max, safe_w_max = self.dynamic_window_safety_stop(dist_obst)

                    if safe_v_max < v and safe_w_max < w:
                        continue
                
                cost = self.alpha * self.heading_cost(theta_next) + self.beta * dist_obst + self.gamma * v

                if cost > max_cost:

                    max_cost = cost
                    best_v, best_w = v, w
        
        # rospy.loginfo(f"dist_obst = {dist_obst}")

        return best_v, best_w 
    
    def heading_cost(self, theta_next):
        """Evaluate the alignment of the robot with the goal.
        
        Args:
            v (float): linear velocity,
            w (float): angular velocity.
        Returns:
            angle (float): angle between the robot heading and the goal angle."""

        # Diference between test angle and heading
        angle_diff = theta_next - self.goal_angle

        # This is necessary to find the smallest angle
        angle_diff = angle_diff % (2 * np.pi)
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi

        angle = np.abs(angle_diff) 
        angle = np.pi - angle

        return angle

    def replan_velocity(self):
        """Method to plan dynamic route.
        
        Args:
            scan (LaserScan): Lidar data.
        Returns:
            best_v (float): best linear velocity,
            best_w (float): best angular velocity."""

        min_v, max_v, min_w, max_w = self.dynamic_window()
        
        best_v, best_w = self.objective_function(min_v, max_v, min_w, max_w)

        return best_v, best_w
        
    # def closest_obstacle_in_central_fov(self, lidar_ranges, fov_positions=300, center_index=320):
    def closest_obstacle_in_central_fov(self, fov_positions=160, center_index=320):
        """
        Return the distance of the closest obstacle in the robot's central field of view (degrees).

        Args:
            lidar_ranges: List of Lidar distance readings.
            fov_positions: Number of positions in the central field of view (75 positions ≈ 30 degrees, 160 positions = 90 degrees).
            center_index: Central index of the Lidar readings list (approximately 320).

        Returns:
            Distance of the closest obstacle in the central field of view, or float('inf') if no obstacles are detected.
        """

        # New index for the angle range considered
        half_fov = fov_positions // 2
        start_index = center_index - half_fov
        end_index = center_index + half_fov

        # Get the distances within the central field of view
        central_fov_distances = self.valid_ranges[start_index:end_index]

        # Filter out "inf" values (no obstacle) and return the smallest valid distance
        valid_distances = [dist for dist in central_fov_distances if dist < float('inf')]
        
        if valid_distances:
            min_distance = min(valid_distances)
            min_index = valid_distances.index(min_distance)

            # Calculate the relative angle between the robot and the closest obstacle
            angle_increment = (self.angle_max - self.angle_min) / len(self.valid_ranges)
            obstacle_angle = (start_index + min_index - center_index) * angle_increment

            return min_distance, obstacle_angle
        else:
            return float('inf'), 0  # No obstacle detected
    
    def average_filter(self, window_size=10):
        """
        Apply average filter to smooth Lidar distance readings.

        Args:
            lidar_ranges: List of Lidar distance readings.
            window_size: Size of the moving average window.

        Returns:
            Readings of smoothed distance.
        """
     
        # The moving average window should be normalized by dividing by the window size
        kernel = np.ones(window_size) / window_size

        # Apply the moving average filter
        smoothed_ranges = np.convolve(self.valid_ranges, kernel, mode='same')

        return smoothed_ranges
        
    def find_free_direction(self, lidar_ranges, robot_width, safety_distance=3.0, center_index=320):
        """
        Procura a faixa livre mais próxima à frente, considerando a distância ao obstáculo e ajustando o
        campo de visão e a quantidade de posições necessárias conforme a proximidade do obstáculo.

        Args:
            lidar_ranges: Lista de leituras de distância do Lidar.
            robot_width: Largura do robô em metros.
            safety_distance: Distância mínima que considera segura para desviar.
            center_index: Índice central do Lidar, que representa a direção à frente.

        Returns:
            O ângulo correspondente à faixa livre mais próxima, ou float('inf') se não encontrar.
        """
        num_positions = len(lidar_ranges)
        angle_increment = (self.angle_max - self.angle_min) / num_positions  # Incremento do ângulo entre leituras do Lidar

        # Ajusta a quantidade de posições necessárias conforme a distância de segurança e a proximidade do obstáculo
        
        if self.closest_obstacle_distance < float('inf'):
            # Ajustar o número de posições necessárias para cobrir a largura do robô com base na distância
            required_positions = int(np.ceil(robot_width / (self.closest_obstacle_distance * angle_increment)))  
        else:
            # Se não houver obstáculos, use uma configuração padrão de distância de segurança
            required_positions = int(np.ceil(robot_width / (safety_distance * angle_increment)))

        # Varre para esquerda e direita a partir do centro
        for offset in range(0, num_positions // 2):
            left_start = center_index - offset - required_positions // 2
            right_start = center_index + offset - required_positions // 2

            # Verifica a faixa à esquerda
            if left_start >= 0 and all([r > safety_distance for r in lidar_ranges[left_start:left_start + required_positions]]):
                return self.angle_min + (left_start + required_positions // 2) * angle_increment
            
            # Verifica a faixa à direita
            if right_start + required_positions < num_positions and all([r > safety_distance for r in lidar_ranges[right_start:right_start + required_positions]]):
                return self.angle_min + (right_start + required_positions // 2) * angle_increment

        # Se não encontrar nenhuma faixa livre, retorna o centro como fallback
        return float('inf')
    
    def is_path_clear(self, lidar_ranges, fov_positions=480, center_index=320):
        """
        Verifica se não há obstáculos ao lado do robô para garantir que o desvio foi completo.

        Args:
            lidar_ranges: Lista de leituras de distância do Lidar.
            safety_distance: Distância mínima que considera segura.
            fov_positions: Número de posições no FOV central (por exemplo, 100 posições).
            center_index: Índice central da lista de leituras do Lidar.

        Returns:
            True se o caminho estiver livre à frente e nas laterais, False se ainda houver obstáculos próximos.
        """
        half_fov = fov_positions // 2
        start_index = max(center_index - half_fov, 0)
        end_index = min(center_index + half_fov, len(lidar_ranges) - 1)

        # Verifica tanto à frente quanto um pouco nas laterais
        for i in range(start_index, end_index):
            if lidar_ranges[i] < self.safety_distance_to_start:
                return False  # Obstáculo ainda próximo, desvio não completo

        return True  # Caminho seguro

    def target_waypoint(self):
        """Calculate the target waypoint in baselink frame.
        """

        goal_baselink_frame = worldToBaselink(
            target_lat=self.current_target.latitude, target_lon=self.current_target.longitude,
            current_location_lat=self.current_location.latitude, current_location_lon=self.current_location.longitude,
            current_yaw=self.current_yaw)
        
        self.goal_distance = np.linalg.norm(goal_baselink_frame)  # [m]
        self.goal_angle = np.degrees(np.arctan2(
            goal_baselink_frame[1], goal_baselink_frame[0]))  # [degrees]

    ############################################################################
    # region MAIN CONTROL LOOP CALLBACK
    ############################################################################

    def laserScanCallback(self, scan) -> None:
        """ We use lidar points to define obstacle in baselink frame and find the available path that is the closest to the waypoint we should travel to.
        We work with two main frames: baselink and world
        Baselink: X forward, Y left of the vehicle. 0~360 degrees, counter-clockwise, 0 is in the back
        World: latlon, so X (lat) points up and Y (lon) points to the right. 0~360 degrees, clockwise, 0 is in positive X 
        """

        # Avoiding the callback if the conditions are not met
        if not scan.ranges or self.current_state.mode == "MANUAL" or not self.current_target or not self.current_location or not self.home_waypoint:
            return

        # Make sure the scan values are valid before doing any math
        self.valid_ranges = np.array(scan.ranges)
        self.valid_ranges[self.valid_ranges == 0] = 1e6

        # Apply average filter to smooth the Lidar readings and reduce noise
        self.valid_ranges = self.average_filter(window_size=5)

        # Calculate the target waypoint in baselink frame
        self.target_waypoint()

        # If we are in AUTO mode we must reset the previous guided point angle
        if self.current_state.mode == "AUTO":
            self.previous_guided_point_angle = None

        # Verify the distance to the closest obstacle for 90 degrees in front of the robot
        closest_in_fov_90, obstacle_angle_90 = self.closest_obstacle_in_central_fov()

        # Verify the distance to the closest obstacle for 270 degrees in front of the robot
        closest_in_fov_270, obstacle_angle_270 = self.closest_obstacle_in_central_fov(fov_positions=480)

        # Verify the distance to the closest obstacle for 120 degrees in front of the robot
        closest_in_fov_120, obstacle_angle_120 = self.closest_obstacle_in_central_fov(fov_positions=213)

        # Verify the distance to the closest obstacle for 180 degrees in front of the robot
        closest_in_fov_180, obstacle_angle_180 = self.closest_obstacle_in_central_fov(fov_positions=320)

        # Verify the distance to the closest obstacle for 220 degrees in front of the robot
        closest_in_fov_220, obstacle_angle_220 = self.closest_obstacle_in_central_fov(fov_positions=391)

        if self.best_v is not None and self.best_w is not None:
            closest_in_fov = closest_in_fov_270
        else:
            # Minimun distance to start the avoidance behavior
            closest_in_fov = closest_in_fov_90
            # 180 seria melhor

        if closest_in_fov < self.safety_distance_to_start:
            self.closest_obstacle_distance = closest_in_fov

            # if self.debug_mode:
            #     rospy.logwarn(
            #         f"Obstacle detected in less than {self.closest_obstacle_distance}m!")
            
            # REPLAN VELOCITY - DWA
            self.best_v, self.best_w = self.replan_velocity()
            if time() - self.last_command_time > self.command_sending_interval:

                # self.dt = time() - self.last_command_time
                # if self.dt > 0.21:
                #     self.dt = 0.15

                self.setFlightMode(mode="GUIDED")
                self.sendGuidedPointLocalFrame(
                    self.best_v, self.best_w)
                self.last_command_time = time()

                #if self.goal_distance < (self.safety_distance_to_start + 0.6):
                if self.goal_distance < self.next_waypoint_dist:
                    rospy.loginfo("Next waypoint")
                    self.advance_to_next_waypoint(self.current_waypoint_index + 1)
        
        else:         
            # Verify if the path is completely free ahead and on the sides, if so, finish the obstacle avoidance
            #if time() - self.last_command_time > 5*self.command_sending_interval and self.current_state.mode != "AUTO" and closest_in_fov_270 > self.safety_distance_to_start:
            if time() - self.last_command_time > 5*self.command_sending_interval and self.current_state.mode != "AUTO" and closest_in_fov_270 > self.safety_distance_to_end:   
                self.best_v = None
                self.best_w = None

                # safe_wp_index = self.find_safe_waypoint()
                # if safe_wp_index is not None:
                #     self.advance_to_next_waypoint(safe_wp_index)

                self.setFlightMode(mode="AUTO")
                self.last_command_time = time()           

            # elif self.best_v is not None and self.best_w is not None:
            #     self.sendGuidedPointLocalFrame(
            #         self.max_v, 0.0)
                
if __name__ == "__main__":
    try:
        obst_avoid_node = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass