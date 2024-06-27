import numpy as np


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


# points_to_test = [[1, 0], [0, 0], [0, 1], [1, 1]]
# length = 2
# width = 0.5
# angle = np.pi / 2
# result = testPointsInRotatedRectangle(points_to_test, length, width, angle)
# print(result)
