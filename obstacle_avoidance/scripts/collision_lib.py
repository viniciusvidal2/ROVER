import numpy as np
import rospy

def createRotatedRectangle(length, width, angle):
    # Define the rectangle corners in the local coordinate system
    half_width = width / 2
    corners = np.array([
        [0, -half_width],
        [length, -half_width],
        [length, half_width],
        [0, half_width]
    ])

    # Create the rotation matrix
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    rotation_matrix = np.array([
        [cos_theta, -sin_theta],
        [sin_theta, cos_theta]
    ])

    # Rotate the corners
    return (rotation_matrix @ corners.T).T


def isPointInTriangle(p, a, b, c):
    v0 = c - a
    v1 = b - a
    v2 = p - a

    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)

    inv_denom = 1 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * inv_denom
    v = (dot00 * dot12 - dot01 * dot02) * inv_denom

    return (u >= 0) and (v >= 0) and (u + v < 1)


def isPointInRotatedRectangle(point, rotated_corners):
    # Check if the point is in either of the two triangles forming the rectangle
    return (isPointInTriangle(point, rotated_corners[0], rotated_corners[1], rotated_corners[2]) or
            isPointInTriangle(point, rotated_corners[0], rotated_corners[2], rotated_corners[3]))


def testPointsInRotatedRectangle(points, length, width, angle):
    rotated_corners = createRotatedRectangle(length, width, angle)
    any_points_inside_rectangle = False
    for point in points:
        if isPointInRotatedRectangle(np.asarray(point), rotated_corners):
            any_points_inside_rectangle = True
            break

    return any_points_inside_rectangle


def createAngleTestSequence(goal_angle, angle_step, full_test_range):
    # Lets have a list of angles to test, starting from the goal angle and going in each direction
    # to try to keep the avoidance behavior, in a best first search manner
    angles = list()  # [RAD]
    angles.append(np.radians(goal_angle))
    for i in range(angle_step, full_test_range, angle_step):
        angles.append(np.radians(goal_angle + i))
    for i in range(angle_step, full_test_range, angle_step):
        angles.append(np.radians(goal_angle - i))

    return angles


def calculateBestTrajectoryGuidedPoint(angle_tests, goal_distance, obstacles_baselink_frame_xy, vehicle_width):
    # Check if the path is clear for each angle, and return the first one that is clear
    path_found = False
    chosen_angle = angle_tests[0]
    for angle in angle_tests:
        if testPointsInRotatedRectangle(obstacles_baselink_frame_xy, length=goal_distance, width=vehicle_width, angle=angle):
            path_found = True
            chosen_angle = angle
            break

    # If we found a path, lets calculate the guided point based on the chosen angle using the goal distance
    if path_found:
        guided_point_x = goal_distance * np.cos(chosen_angle)
        guided_point_y = goal_distance * np.sin(chosen_angle)
        return [guided_point_x, guided_point_y], np.degrees(abs(chosen_angle - angle_tests[0]))
    else:
        return [0, 0], np.degrees(abs(angle_tests[-1] - angle_tests[0]))
    

def checkSafeFOV(obstacles_baselink_frame, goal_baselink_frame):
        # Check if there is enough FOV to the goal before changing to AUTO mode, which means we have
        # now quite left the obstacle behind
        safe_fov = 60
        for _, angle in obstacles_baselink_frame:
            if abs(goal_baselink_frame[1] - angle) < safe_fov/2:
                return False
            
        return True

# points_to_test = [[1, 0], [0, 0], [0, 1], [1, 1]]
# length = 2
# width = 0.5
# angle = np.pi / 2
# result = testPointsInRotatedRectangle(points_to_test, length, width, angle)
# print(result)
