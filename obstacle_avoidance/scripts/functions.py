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