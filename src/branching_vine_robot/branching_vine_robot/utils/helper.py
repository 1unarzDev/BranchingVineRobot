import numpy as np

def get_angle_2d(origin, point, reference_direction=(1,0)):
    """
    Computes the angle between a reference direction and the vector from origin to point in a 2D plane.

    Parameters:
        origin (tuple): (x, y) coordinates of the reference point (e.g., camera position).
        point (tuple): (x, y) coordinates of the target point.
        reference_direction (tuple): (dx, dy) direction vector defining the reference (e.g., (1,0) for x-axis).

    Returns:
        float: Angle in radians (counterclockwise from reference_direction).
    """
    dx, dy = np.array(point) - np.array(origin)
    angle = np.arctan2(dy, dx) - np.arctan2(reference_direction[1], reference_direction[0])

    # Normalize angle to the range (-pi, pi]
    angle = (angle + np.pi) % (2 * np.pi) - np.pi

    return angle

def get_angle_3d(point1, point2):
    """
    Computes the azimuth (theta) and elevation (phi) angles from point1 to point2 in 3D space.

    Parameters:
        point1 (tuple): (x1, y1, z1) coordinates of the reference point (e.g., camera position).
        point2 (tuple): (x2, y2, z2) coordinates of the target point.

    Returns:
        tuple: (theta, phi) in radians
            - theta (azimuth): Angle in the XY plane, counterclockwise from the positive X-axis.
            - phi (elevation): Angle from the XY plane upwards.
    """
    # Compute difference vector
    dx, dy, dz = np.array(point2) - np.array(point1)

    # Compute azimuth (theta) in the XY plane
    theta = np.arctan2(dy, dx)  # Counterclockwise from the positive X-axis

    # Compute elevation (phi) angle
    r = np.sqrt(dx**2 + dy**2 + dz**2)  # Euclidean distance
    phi = np.arcsin(dz / r)  # Angle above the XY plane

    return theta, phi