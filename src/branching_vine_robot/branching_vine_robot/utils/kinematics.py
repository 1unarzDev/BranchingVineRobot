from branching_vine_robot.config import MAX_TIP_SPEED, APPROX_ACTUATOR_SPEED_TO_ANGLE

def calc_actuator(dist, theta, phi, actuator_shift):
    """
    Parameters:
        - dist (float) The distance from the robot(0, 0, 0) at the given angle in spherical coordinates 
        - theta (float) Azimuth radians
        - phi (float) Elevation radians
        - actuator shift (float) Shift of actuators from default position (-pi/2, pi/6, 5pi/6)
        
    Returns:
        a1 (float) The speed of the first actuator (originally at pi/6)
        a2 (float) The speed of the second actuator (originally at 5pi/6)
        a3 (float) The speed of the third actuator
        a4 (float) The angle of the valve

    Where a4 is the amount the ball valve is opened, while
    a1, a2, and a3 are the forces of the actuators
    """