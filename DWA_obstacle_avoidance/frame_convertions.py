#!/usr/bin/env python3
import numpy as np
from typing import Tuple
import utm


def rotationMatrix(angle: float) -> np.ndarray:
    """Creates a 2x2 rotation matrix from an angle

    Args:
        angle (float): rotation angle [RAD]

    Returns:
        np.ndarray: 2x2 rotation matrix
    """
    ca = np.cos(angle)
    sa = np.sin(angle)

    return np.array([[ca, -sa], [sa, ca]])


def latLonToUtm(lat: float, lon: float) -> Tuple[float, float, int, str]:
    """Using lat lon coordinates and utm lib to convert to UTM coordinates

    Args:
        lat (float): latitude
        lon (float): longitude

    Returns:
        float, float, int, str: utm easting, utm northing, zone number, zone letter
    """
    utm_e, utm_n, zn, zl = utm.from_latlon(latitude=lat, longitude=lon)

    return utm_e, utm_n, zn, zl


def utmToLatLon(utm_e: float, utm_n: float, zn: int, zl: str) -> Tuple[float, float]:
    """Uses UTM values to obtain latlon coordinates with the utm lib

    Args:
        utm_e (float): utm easting
        utm_n (float): utm northing
        zn (int): zone number
        zl (str): zone letter

    Returns:
        float, float: latitude and longitude coordinates
    """
    lat, lon = utm.to_latlon(
        easting=utm_e, northing=utm_n, zone_number=zn, zone_letter=zl)

    return lat, lon


def laserScanToXY(range: float, angle: float) -> np.ndarray:
    """Converts a laser reading in range and angle to xy in baselink frame

    Args:
        range (float): reading range [m]
        angle (float): reading angle [RAD]

    Returns:
        np.ndarray: point in baselink frame [x, y]
    """
    x_baselink = range*np.cos(angle)
    y_baselink = range*np.sin(angle)

    return np.array([x_baselink, y_baselink])


def baselinkToWorld(xy_baselink: np.ndarray, current_lat: float, current_lon: float, current_yaw: float) -> Tuple[float, float]:
    """Converts coordinates from baselink frame [x, y] to global frame [latlon]

    Args:
        xy_baselink (np.ndarray): point in baselink frame [x, y]
        current_lat (float): vehicle latitude in degrees
        current_lon (float): vehicle longitude in degrees
        current_yaw (float): current vehicle yaw angle [RAD]

    Returns:
        Tuple[float, float]: point latitude and longitude
    """
    # Get current location from latlon to UTM coordinates
    utm_east, utm_north, zn, zl = latLonToUtm(
        lat=current_lat, lon=current_lon)
    # Create rotation from baselink to world based on the current yaw and apply
    world_angle_baselink = np.pi/2 - current_yaw
    world_R_baselink = rotationMatrix(angle=world_angle_baselink)
    d_utm = world_R_baselink @ xy_baselink
    # Add to the current location in UTM
    utm_output = np.array([utm_east, utm_north]) + d_utm

    return utmToLatLon(utm_e=utm_output[0], utm_n=utm_output[1], zn=zn, zl=zl)


def worldToBaselink(target_lat: float, target_lon: float, current_location_lat: float, current_location_lon: float, current_yaw: float) -> np.ndarray:
    """Converts a point from global frame [latitude and longitude] to baselink frame [x, y]

    Args:
        target_lat (float): target point latitude
        target_lon (float): target point longitude
        current_location_lat (float): vehicle location latitude
        current_location_lon (float): vehicle location longitude
        current_yaw (float): vehicle yaw angle [RAD]

    Returns:
        np.ndarray: target point in baselink frame [x, ]
    """
    # Get current location from latlon to UTM coordinates
    utm_east, utm_north, _, _ = latLonToUtm(
        lat=current_location_lat, lon=current_location_lon)
    # Get the target location from latlon to UTM coordinates
    utm_target_east, utm_target_north, _, _ = latLonToUtm(
        lat=target_lat, lon=target_lon)
    # Calculate the offset from the current location to the target location in UTM frame
    d_utm = np.array([utm_target_east - utm_east,
                     utm_target_north - utm_north])
    # Create rotation from world to baselink based on the current yaw and apply
    baselink_angle_world = current_yaw - np.pi/2
    baselink_R_world = rotationMatrix(angle=baselink_angle_world)
    target_baselink_frame = baselink_R_world @ d_utm

    return target_baselink_frame
