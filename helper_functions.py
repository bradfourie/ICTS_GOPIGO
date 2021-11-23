from classes import *
from constants import *
import time

def setup_bot(gpg, servo, *args):
    """
    setup_bot sets up the GoPiGo3 based on the control paradigm, if the control paradigm is not specified in the
    optional argument, then it resets the GoPiGo3 by resetting it, stopping it, setting its speed to the default value
    and resetting the servo motor.

    :param gpg: The GoPiGo3 instance
    :param servo: The servo motor instance
    :param args: Optional argument storing the current control paradigm
    :return: None
    """
    try:
        current_control_paradigm = args[0]
    except:
        current_control_paradigm = None

    if current_control_paradigm == ControlParadigm.MODE_FREE_ROAMING:
        gpg.set_speed(ROBOT_SPEED)
        servo.rotate_servo(ANGLE_STRAIGHT_AHEAD)
    elif current_control_paradigm == ControlParadigm.MODE_FOLLOW_LEFT_WALL:
        gpg.turn_degrees(-TURN_DEGREES)
        time.sleep(SHORT_WAIT_TIME)
        servo.rotate_servo(ANGLE_STRAIGHT_AHEAD)
    elif current_control_paradigm == ControlParadigm.MODE_FOLLOW_RIGHT_WALL:
        gpg.turn_degrees(TURN_DEGREES)
        time.sleep(SHORT_WAIT_TIME)
        servo.rotate_servo(ANGLE_STRAIGHT_AHEAD)
    elif current_control_paradigm == ControlParadigm.MODE_USER_CONTROL:
        gpg.set_speed(ROBOT_SPEED)
        servo.rotate_servo(ANGLE_STRAIGHT_AHEAD)
    else:
        gpg.reset_all()
        servo.reset_servo()
        gpg.stop()
        gpg.set_speed(ROBOT_SPEED)


def update_robot_velocity(gpg, distance_mm):
    """
    update_robot_velocity uses the distance between the GoPiGo3 and the object closest to it and calculates a new
    velocity based on the amount of free space in front of the GoPiGo3. The GoPiGo3 is then updated with this new
    velocity.

    :param gpg: The GoPiGo3 instance
    :param distance_mm: The amount of free space in front of the GoPiGo3 measured in mm
    :return: None
    """
    # Calculate the new velocity to be proportional to the distance from the closest object
    new_velocity = VELOCITY_GRADIENT * distance_mm + VELOCITY_CONSTANT
    # Clamp the velocity for the upper and lower bounds
    if distance_mm < MIN_DISTANCE:
        gpg.stop()
    elif distance_mm > MAX_DISTANCE:
        gpg.set_speed(MAX_VELOCITY)
        gpg.forward()
    else:
        gpg.set_speed(new_velocity)
        gpg.forward()


def avoid_obstacle(gpg, distance_sensor, servo):
    """
    avoid_obstacle scans the surroundings of the GoPiGo3 and finds the direction with the most available free space.
    Using this direction and the amount of free space, the GoPiGo3's velocity and heading is then updated using the
    update_robot_velocity function.

    :param gpg: The GoPiGo3 instance
    :param distance_sensor: The distance sensor instance
    :param servo: The servo motor instance
    :return: None
    """
    gpg.stop()
    max_dist = 0
    min_dist = 10000
    new_direction = 0
    for num in range(NUM_ANGLES + 1):
        distance_mm = distance_sensor.read_mm()
        if distance_mm > max_dist:
            max_dist = distance_mm
            new_direction = num * 180 / NUM_ANGLES
        if distance_mm < min_dist:
            min_dist = distance_mm
            update_robot_velocity(gpg, distance_mm)
        # move servo to designated position
        servo.rotate_servo(num * 180 / NUM_ANGLES)
        time.sleep(SERVO_WAIT_TIME)

    new_direction_gpg = 90 - new_direction

    gpg.set_speed(ROBOT_SPEED)
    gpg.turn_degrees(new_direction_gpg)
    time.sleep(SHORT_WAIT_TIME)
    gpg.forward()


def pid_controller_update(distance_sensor, pid_controller, gpg, is_right):
    """
    pid_controller_update performs a single update of the PID controller when following a wall. Whether the PID
    controller should update for following the left or right wall is specified in the is_right argument. The PID
    controller takes as input the measured distance in front of the robot and outputs a corrective factor which is
    subtracted from the speed of the left motor and added to the speed of the right motor (when following the left
    wall), and vice versa for when the robot is following the right wall.

    :param distance_sensor: The distance sensor instance
    :param pid_controller: The PID object instance
    :param gpg: The GoPiGo3 robot instance
    :param is_right: Boolean argument specifying if the robot should follow the right wall
    :return: correction
    """
    # corrective factor that switches the signs if the robot has to follow the right wall
    factor = 1
    if is_right:
        factor = -1

    distance = distance_sensor.read_mm()

    pid_controller.update(distance)
    correction = pid_controller.output

    left_motor_speed = int(ROBOT_SPEED + (correction * factor))
    right_motor_speed = int(ROBOT_SPEED - (correction * factor))

    gpg.set_motor_dps(gpg.MOTOR_LEFT, dps=left_motor_speed)
    gpg.set_motor_dps(gpg.MOTOR_RIGHT, dps=right_motor_speed)

    return correction
