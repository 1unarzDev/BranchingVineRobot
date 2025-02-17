""" PARAMS """ 
MAX_DEPTH = 5000
MIN_DEPTH = 1000
DEPTH_WIDTH = 640 # px
DEPTH_HEIGHT = 480 # px
RGB_WIDTH = 960 # px
RGB_HEIGHT = 540 # px
CAM_FPS = 30 # frames/s
DIST_THRESHOLD = 5000 # mm DEPRECATED

""" MEASURED """ 
MAX_TIP_SPEED = ... # m/s
APPROX_ACTUATOR_SPEED_TO_ANGLE = ... # Approximation for the ratio of the speed of the spool rotation to the change in angle
LOOKAHEAD_DIST = 0.1 # m
# All starting motors can be matched to the corresponding robot by adding the robot number (starting at 0)
PNEUMATIC_CONTROL_VALVE = 12
A1 = 0
A2 = 4
A3 = 8