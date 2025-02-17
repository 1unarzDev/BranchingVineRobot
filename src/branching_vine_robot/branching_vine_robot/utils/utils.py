import numpy as np

def get_angle_2d(origin, point, reference_direction=(1,0)):
    """
    Computes the angle between a reference direction and the vector from origin to point in a 2D plane.

    Parameters:
        - origin (Tuple(float, float)) - (x, y) coordinates of the reference point
        - point (Tuple(float, float)) - (x, y) coordinates of the target point.
        - reference_direction (Tuple(float, float)) - (dx, dy) direction vector defining the reference (e.g., (1,0) for x-axis).

    Output:
        - angle (float) - Angle in radians (counterclockwise from reference_direction).
    """
    
    # Calculate the angle between the points by calculating displacement and using inverse tangent
    dx, dy = np.array(point) - np.array(origin)
    angle = np.arctan2(dy, dx) - np.arctan2(reference_direction[1], reference_direction[0])

    # Normalize angle to the range (-pi, pi]
    angle = (angle + np.pi) % (2 * np.pi) - np.pi

    return angle

def get_angle_3d(p1, p2):
    """
    Computes the azimuth (theta) and elevation (phi) angles from p1 to p2 in 3D space.

    Parameters:
        p1 (Tuple) - (x1, y1, z1) coordinates of the reference point.
        p2 (Tuple) - (x2, y2, z2) coordinates of the target point.

    Returns:
        theta/azimuth (float) - Angle in the XY plane, counterclockwise from the positive X-axis.
        phi/elevation (float) - from the XY plane upwards.
    """
    # Compute difference vector
    dx, dy, dz = np.array(p2) - np.array(p1)

    # Compute azimuth (theta) in the XY plane counterclockwise from the positive X-axis
    theta = np.arctan2(dy, dx)  

    # Compute elevation (phi) angle
    r = np.sqrt(dx**2 + dy**2 + dz**2) 
    phi = np.arcsin(dz / r) 

    return theta, phi