""" PARAMS """ 
DIST_THRESHOLD = 40 # m
DEPTH_WIDTH = 480 # px
DEPTH_HEIGHT = 270 # px
RGB_WIDTH = 640 # px
RGB_HEIGHT = 480 # px
CAM_FPS = 40 # frames/s

""" MEASURED """ 
MAX_TIP_SPEED = ... # m/s
APPROX_ACTUATOR_SPEED_TO_ANGLE = ... # Approximation for the ratio of the speed of the spool rotation to the change in angle
LOOKAHEAD_DIST = 0.1 # m
# All starting motors can be matched to the corresponding robot by adding the robot number (starting at 0)
PNEUMATIC_CONTROL_VALVE = 12
A1 = 0
A2 = 4
A3 = 8