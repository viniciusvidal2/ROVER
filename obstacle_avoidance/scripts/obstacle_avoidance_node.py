#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State, GlobalPositionTarget, WaypointReached
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import  Float64
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import tf
from time import time
from pyproj import Proj, transform
from scipy.spatial.transform import Rotation as R


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node("obstacle_avoidance_node", anonymous=False)
        
        # Our default projection pattern
        self.wgs84 = Proj(init='epsg:4326')
        self.utm_zone = 23 # TODO: make this adjustable
        self.utm_proj = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')
        
        # Variables
        self.current_yaw = 0.0 # [RAD]
        self.current_location = None # GPS data
        self.original_target = None # target waypoint data in AUTO mode
        self.avoiding = False # control if we are in a scenario to run the algorithm or not
        self.last_input_scan_message_time = time() # control the node execution incoming messages
        self.current_state = State() # vehicle driving mode
        self.original_mode = "" # original mode name
        self.max_obstacle_distance = 10 # [m]
        self.K = -0.75 # potential fields repulsive force gain

        # Subscribers to mavros and laserscan messages
        rospy.Subscriber("/livox/scan", LaserScan, self.laserScanCallback)
        rospy.Subscriber("/mavros/setpoint_raw/target_global", GlobalPositionTarget, self.targetPointCallback)
        rospy.Subscriber("/mavros/state", State, self.stateCallback)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gpsCallback)
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compassCallback)

        # If no message for some reason, resume original state with a frequency based callback
        self.timer_cb = rospy.Timer(rospy.Duration(1.0), self.travelStateCheckCallback)

        # Publishers
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)

    ############################################################################
    ### FRAME CONVERTION METHODS
    ############################################################################
    def latLonToUtm(self, lat, lon):
        utm_e, utm_n = transform(self.wgs84, self.utm_proj, lon, lat)

        return utm_e, utm_n

    def utmToLatLon(self, utm_e, utm_n):
        lon, lat = transform(self.utm_proj, self.wgs84, utm_e, utm_n)
        
        return lat, lon
    
    def laserScanToXY(self, range, angle):
        x_baselink = range*np.cos(angle)
        y_baselink = range*np.sin(angle)

        return x_baselink, y_baselink

    def baselinkToWorld(self, x_baselink, y_baselink):
        # Get current location from latlon to UTM coordinates
        utm_east, utm_north = self.latLonToUtm(lat=self.current_location.lat, lon=self.current_location.lon)
        # Create rotation from baselink to world based on the current yaw and apply
        world_R_baselink = R.from_euler('z', np.pi/2 - self.current_yaw)
        d_utm = world_R_baselink.apply(np.array([x_baselink, y_baselink]))
        # Add to the current location in UTM
        utm_output = np.array([utm_east, utm_north]) + d_utm

        return self.utmToLatLon(utm_e=utm_output[0], utm_n=utm_output[1])

    def worldToBaselink(self, lat, lon):
        pass

    ############################################################################
    ### SENSOR CALLBACKS
    ############################################################################
    def targetPointCallback(self, data):
        if not self.avoiding:
            self.original_target = data

    def stateCallback(self, state):
        # Update the mode we are travelling
        self.current_state = state
        # If not avoiding yet, we want to record which mode we were using to return to it afterwards
        if not self.avoiding:
            self.original_mode = self.current_state.mode

    def gpsCallback(self, data):
        self.current_location = data

    def compassCallback(self, data):
        # Convert compass heading from degrees to radians
        self.current_yaw = np.radians(data.data) # [RAD]

    def travelStateCheckCallback(self, event):
        # Check if we are some time with no input data, and if so get back to AUTO mode mission
        if self.avoiding and time() - self.last_input_scan_message_time > 1:
            rospy.logwarn("No data was received in the last 1 second. Returning to original state ...")
            self.resumeOriginalState()
            self.avoiding = False

    ############################################################################
    ### CONTROL FUNCTIONS
    ############################################################################
    def setFlightMode(self, mode):
        self.original_mode = self.current_state.mode
        try:
            rospy.wait_for_service('/mavros/set_mode')
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Changing navigation mode to {mode}...")
                while self.current_state.mode != mode:
                    response = set_mode(custom_mode=mode)
                    rospy.loginfo("Wayting for mode change request confirmation ...")
                    rospy.sleep(0.1)
                rospy.loginfo(f"Mode {mode} activated.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to change navigation mode to {mode}: {e}")

    def resumeOriginalState(self):
        # Try to get back to AUTO mode after obstacle is avoided
        # If AUTO, resume mission where it left of
        # If any errors, get back to manual mode
        if self.original_mode and self.original_mode == "AUTO":
            try:
                self.setFlightMode(self.original_mode)
                if self.original_mode == "AUTO" and self.original_wp_index is not None:
                    rospy.loginfo(f"Resuming mission in waypoint {self.original_wp_index} in AUTO mode.")
                # elif self.original_mode == "GUIDED" and self.original_target:
                #     self.setpoint_pub.publish(self.original_target)
                #     rospy.loginfo(f"Retomando movimento para o alvo original em GUIDED: {self.original_target}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to resume original state: {e}")
        else:
            rospy.logerr("Failed to get back to AUTO mode, setting MANUAL mode.")
            self.setFlightMode("MANUAL")

    def calculateGoalDirectionBaselinkFrame(self, target_location_world_frame, current_yaw_world_frame):
        pass

    def calculateForces(self, obstacles_baselink_frame, goal_direction_baselink_frame):
        # We have a normalized repulsive force
        attractive_force = np.array(goal_direction_baselink_frame)/np.linalg.norm(goal_direction_baselink_frame)
        # Each obstacle is a combination of ditance and angle (in radians), and adds some importance to repulsive force
        # The repulsive force must have the opposite direction of the angle unit vector itself from the vehicle to the obstacle
        repulsive_force = np.array([0.0, 0.0])
        for obstacle_distance, angle in obstacles_baselink_frame:
            repulsion_strength = (1 / max(obstacle_distance, 1))**2# * np.sqrt(obstacle_size)
            repulsive_force -= repulsion_strength * np.array([np.cos(angle), np.sin(angle)])
        # Normalize and apply a gain to the the repulsive force
        if np.linalg.norm(repulsive_force) > 0:
            repulsive_force = self.K * repulsive_force / np.linalg.norm(repulsive_force)

        return attractive_force + repulsive_force

    def travelDirectionBaselinkToWorldFrame(self, force_baselink_frame):
        pass

    ############################################################################
    ### MAIN CONTROL LOOP CALLBACK
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
        if not scan.ranges or self.current_state.mode == "MANUAL" or not self.original_target or not self.current_location:
            return
        
        # Make sure the scan values are valid before doing any math
        valid_ranges = np.array(scan.ranges)
        valid_ranges[valid_ranges == 0] = 1e6
        closest_obstacle_distance_index = np.argmin(valid_ranges)
        closest_obstacle_distance = valid_ranges[closest_obstacle_distance_index]
        
        # If any point is close enough, process the avoidance behavior
        if closest_obstacle_distance < self.max_obstacle_distance:
            rospy.logerr(f"Obstáculo detectado a menos de {self.max_obstacle_distance}m!")
            # Start avoiding and set the GUIDED mode to send commands
            avoiding = True
            if self.current_state.mode == "AUTO":
                self.setFlightMode("GUIDED")
            # In case there is next point in the mission that we can use as goal, we can calculate the avoidance
            if self.original_target:
                goal_baselink_frame = self.worldToBaselink(lat=self.original_target.lat, lon = self.original_target.lon)
                goal_angle = np.arctan2(goal_baselink_frame[1], goal_baselink_frame[0])
                # Isolate the readings that return the obstacles
                obstacles_baselink_frame = [[r, (i - len(valid_ranges)/2) * scan.angle_increment] for i, r in enumerate(valid_ranges) if r < self.max_obstacle_distance]
                obstacles_baselink_frame_xy = [self.laserScanToXY(range=r, angle=a) for r, a in obstacles_baselink_frame]
                # Calculate total force
                total_force = self.calculateForces(obstacles_baselink_frame=obstacles_baselink_frame, goal_direction_baselink_frame=goal_baselink_frame)
                # Create the new travel point in baselink frame

                # Convert the travel point to world frame

                # Send the new point to the vehicle

                goal_direction = calculate_goal_direction(current_location, original_target, current_yaw)
                goal_angle = np.arctan2(goal_direction[1], goal_direction[0])
                publish_goal_direction(goal_angle, scan.header.frame_id)                
                total_force = calculate_forces(valid_ranges, scan.angle_increment, goal_direction)
                magnitude, desvio_angle = np.linalg.norm(total_force), np.arctan2(total_force[1], total_force[0])
                x = magnitude * np.cos(current_yaw - desvio_angle) * change_factor 
                y = magnitude * np.sin(current_yaw - desvio_angle) * change_factor
                new_lat = current_location.latitude + x
                new_lon = current_location.longitude + y
                new_alt = current_location.altitude
                new_location = GlobalPositionTarget()
                new_location.latitude = new_lat
                new_location.longitude = new_lon
                new_location.altitude = new_alt
                rospy.loginfo(f"original_target: Lat: {original_target.latitude:.6f}, Lon: {original_target.longitude:.6f}, Alt: {original_target.altitude:.2f}")            
                rospy.loginfo(f"Movendo para nova localização: Lat: {new_lat:.6f}, Lon: {new_lon:.6f}, Alt: {new_alt:.2f}")
                setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
                setpoint_pub.publish(new_location)
                publish_debug_markers(valid_ranges, scan.angle_increment, scan.angle_min, desvio_angle, scan.header.frame_id)
        elif avoiding:
            rospy.logwarn("No obstacle observed nearby, resuming original mission ...")
            resume_original_state()
            avoiding = False


if __name__ == "__main__":
    try:
        obst_avoid_node = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass
