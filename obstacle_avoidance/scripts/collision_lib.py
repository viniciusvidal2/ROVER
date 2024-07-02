import numpy as np
from frame_convertions import rotationMatrix


def areRotatedPointsInRectangle(points, length, width, angle):
    R = rotationMatrix(angle)
    for point in points:
        rotated_point = R @ np.array(point)
        if 0 < rotated_point[0] < length and -width/2 < rotated_point[1] < width/2:
            return True

    return False


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
    for angle in angle_tests:
        if not areRotatedPointsInRectangle(obstacles_baselink_frame_xy, length=goal_distance, width=vehicle_width, angle=-angle):
            guided_point_x = goal_distance * np.cos(angle)
            guided_point_y = goal_distance * np.sin(angle)
            return [guided_point_x, guided_point_y], np.degrees(abs(angle - angle_tests[0]))

    return [0, 0], np.degrees(abs(angle_tests[-1] - angle_tests[0]))


def checkSafeFOV(obstacles_baselink_frame, goal_angle_baselink_frame):
    # Check if there is enough FOV to the goal before changing to AUTO mode, which means we have
    # now quite left the obstacle behind
    safe_fov = 90
    for _, angle in obstacles_baselink_frame:
        if abs(goal_angle_baselink_frame - angle) < safe_fov/2:
            return False

    return True

# points_to_test = [[1, 0], [0, 0], [0, 1], [1, 1]]
# length = 2
# width = 0.5
# angle = np.pi / 2
# result = arePointsInRotatedRectangle(points_to_test, length, width, angle)
# print(result)
