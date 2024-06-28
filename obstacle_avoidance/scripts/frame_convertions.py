#!/usr/bin/env python3
import numpy as np
import utm


def rotationMatrix(angle):
    ca = np.cos(angle)
    sa = np.sin(angle)

    return np.array([[ca, -sa], [sa, ca]])


def latLonToUtm(lat, lon):
    utm_e, utm_n, zn, zl = utm.from_latlon(lat, lon)

    return utm_e, utm_n, zn, zl


def utmToLatLon(utm_e, utm_n, zn, zl):
    lat, lon = utm.to_latlon(utm_e, utm_n, zn, zl)

    return lat, lon


def laserScanToXY(range, angle):
    x_baselink = range*np.cos(angle)
    y_baselink = range*np.sin(angle)

    return x_baselink, y_baselink


def baselinkToWorld(xy_baselink, current_location, current_yaw, zn, zl):
    # Get current location from latlon to UTM coordinates
    utm_east, utm_north, _, _ = latLonToUtm(
        lat=current_location.latitude, lon=current_location.longitude)
    # Create rotation from baselink to world based on the current yaw and apply
    world_angle_baselink = np.pi/2 - current_yaw
    world_R_baselink = rotationMatrix(world_angle_baselink)
    d_utm = world_R_baselink @ xy_baselink
    # Add to the current location in UTM
    utm_output = np.array([utm_east, utm_north]) + d_utm

    return utmToLatLon(utm_e=utm_output[0], utm_n=utm_output[1], zn=zn, zl=zl)


def worldToBaselink(target_lat, target_lon, current_location, current_yaw):
    # Get current location from latlon to UTM coordinates
    utm_east, utm_north, _, _ = latLonToUtm(
        lat=current_location.latitude, lon=current_location.longitude)
    # Get the target location from latlon to UTM coordinates
    utm_target_east, utm_target_north, _, _ = latLonToUtm(
        lat=target_lat, lon=target_lon)
    # Calculate the offset from the current location to the target location in UTM frame
    d_utm = np.array([utm_target_east - utm_east,
                     utm_target_north - utm_north])
    # Create rotation from world to baselink based on the current yaw and apply
    baselink_angle_world = current_yaw - np.pi/2
    baselink_R_world = rotationMatrix(baselink_angle_world)
    target_baselink_frame = baselink_R_world @ d_utm

    return target_baselink_frame[0], target_baselink_frame[1]
