import numpy as np
from typing import List, Tuple
from frame_convertions import rotationMatrix


def areRotatedPointsInRectangle(points: list, length: float, width: float, angle: float) -> bool:
    """Checks if some points, rotated by an angle, fall inside a rectangle that is supposed to be the path corridor

    Args:
        points (list): list of points [x, y] in baselink frame
        length (float): length [m]
        width (float): width [m]
        angle (float): angle to rotate the points (or the rectangle, depending on the point of view) [RAD]

    Returns:
        bool: True if points fall inside the rectangle
    """
    R = rotationMatrix(angle=angle)
    rotated_points = R @ np.array(points).T
    
    # Check if the rotated points are inside the rectangle, doing each for each collumn at once
    return np.all(np.logical_and(rotated_points[0] > 0, rotated_points[0] < length, np.abs(rotated_points[1]) < width/2))


def createAngleTestSequence(goal_angle: float, angle_step: float, full_test_range: float, start_side: str) -> List[float]:
    """Returns the angles we will test for a free path, in order

    Args:
        goal_angle (float): original angle to the goal
        angle_step (float): step to generate new direction angles
        full_test_range (float): last angle value to test a trajectory
        start_side (str): side to start the search, l or r for left or right

    Returns:
        List[float]: angles to create directions and test [RAD]
    """
    # Lets have a list of angles to test, starting from the goal angle and going in each direction
    # to try to keep the avoidance behavior, in a best first search manner
    angles = list()  # [RAD]
    angles.append(np.radians(goal_angle))
    if start_side == 'l':
        for i in range(angle_step, full_test_range, angle_step):
            angles.append(np.radians(goal_angle + i))
        for i in range(angle_step, full_test_range, angle_step):
            angles.append(np.radians(goal_angle - i))
    elif start_side == 'r':
        for i in range(angle_step, full_test_range, angle_step):
            angles.append(np.radians(goal_angle - i))
        for i in range(angle_step, full_test_range, angle_step):
            angles.append(np.radians(goal_angle + i))

    return angles


def calculateBestTrajectoryGuidedPoint(angle_tests: list, point_distance: float, obstacles_baselink_frame_xy: list, corridor_width: float) -> Tuple[np.ndarray, float]:
    """Check if the path is clear for each angle, and return the first one that is clear

    Args:
        angle_tests (list): angles to generate trajectories with [RAD]
        point_distance (float): distance in the baselink frame to the goal, in meters
        obstacles_baselink_frame_xy (list): list of obstacle in baselink frame [x, y]
        corridor_width (float): width of the corridor we will generate to test for collisions [meters]

    Returns:
        Tuple[np.ndarray, float]: guided waypoint in baselink frame, and angle to the original goal [degrees]
    """
    for angle in angle_tests:
        if not areRotatedPointsInRectangle(points=obstacles_baselink_frame_xy, length=point_distance, width=corridor_width, angle=-angle):
            guided_point_x = point_distance * np.cos(angle)
            guided_point_y = point_distance * np.sin(angle)
            return np.array([guided_point_x, guided_point_y]), np.degrees(angle - angle_tests[0])

    return np.array([0, 0]), np.degrees(angle_tests[-1] - angle_tests[0])


def checkSafeFOV(obstacles_baselink_frame_ra: list, goal_angle_baselink_frame: float) -> bool:
    """Check if we have already a nice view to the original goal, with no nearby obstacles, so then we can change to AUTO again

    Args:
        obstacles_baselink_frame_ra (list): list of obstacles in baselink frame as [distance (r), angle(a, radians)]
        goal_angle_baselink_frame (float): original goal angle in baselink frame [radians]

    Returns:
        bool: if we can go back to AUTO or not
    """
    safe_fov = np.radians(140)
    for _, angle in obstacles_baselink_frame_ra:
        if abs(goal_angle_baselink_frame - angle) < safe_fov/2:
            return False

    return True

# points_to_test = [[1, 0], [0, 0], [0, 1], [1, 1]]
# length = 2
# width = 0.5
# angle = np.pi / 2
# result = arePointsInRotatedRectangle(points_to_test, length, width, angle)
# print(result)
