# Initialise the parameters of the server
SERVER_PORT = 12069

# Initialise the parameters of the robot
ROBOT_SPEED = 300 #300
NUM_ANGLES = 6
ANGLE_STRAIGHT_AHEAD = 90
ANGLE_SLIGHT_LEFT = 135
ANGLE_SLIGHT_RIGHT = 45
MIN_DISTANCE = 300
MAX_DISTANCE = 3000
MAX_VELOCITY = 1000
MIN_VELOCITY = 0
MINIMUM_BATTERY_VOLTAGE = 9

# Initialise the velocity stuff
VELOCITY_GRADIENT = (MAX_VELOCITY- MIN_VELOCITY)/(MAX_DISTANCE-MIN_DISTANCE)
VELOCITY_CONSTANT = MAX_VELOCITY - VELOCITY_GRADIENT*MAX_DISTANCE

# Initialise the PID controller constants
# Tuning method explained in the slides
Kp = 0.8
Ki = 0.6
Kd = 0.4
DESIRED_DISTANCE = 300
THRESHOLD = 800
SAMPLING_TIME = 0.1

# Timer values
SERVO_WAIT_TIME = 0.2
SHORT_WAIT_TIME = 0.08

# User control constants
TURN_DEGREES = 30
MOVE_TIME = 1