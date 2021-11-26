from __future__ import print_function
from __future__ import division

import atexit
import sys
import easygopigo3 as easy
from threading import Thread, Event
from classes import *
from helper_functions import *
from constants import *
import PID
import matplotlib.pyplot as plt

try:
    # Initialise the EasyGoPiGo3 instance and reset all parameters
    gpg = easy.EasyGoPiGo3()
    servo = gpg.init_servo()
    distance_sensor = gpg.init_distance_sensor()
    setup_bot(gpg, servo)

    # When program is stopped ensure that the robot stops
    atexit.register(gpg.stop())
    atexit.register(servo.rotate_servo(ANGLE_STRAIGHT_AHEAD))

    # Check the GoPiGo Battery Voltage
    if gpg.volt() < MINIMUM_BATTERY_VOLTAGE:
        print(" Current GoPiGo battery voltage is critically low: %0.02fV\n The program will now exit... "
              "\n Please recharge or replace the batteries before trying again..." % (gpg.volt()))
    else:
        print(" Current GoPiGo battery voltage is:  %0.02fV" % (gpg.volt()))

    # Set the current control method the GoPiGo will use on startup
    current_control_method = ControlMethod.MODE_FREE_ROAMING

    # Initialise the client-server connection
    server = Server()
    server.connect()

    # Initialise the PID controller - we use a third party library for this
    pid_controller = PID.PID(Kp, Ki, Kd)
    pid_controller.SetPoint = DESIRED_DISTANCE
    pid_controller.setSampleTime(SAMPLING_TIME)

except IOError as error:
    print("The GoPiGo Robot or its Distance Sensor was not detected")

def process_user_commands(string_input):
    """
    process_user_commands receives the user commands from the GUI on the client and sets the appropriate event which
        then triggers certain logic to be executed on the main robot logic thread.

    :param string_input: the command sent from the GUI as a string
    :return: None
    """
    global change_cp_user_control, change_cp_left_wall, change_cp_right_wall, change_cp_free_roaming
    global robot_move_forward, robot_move_left, robot_move_back, robot_move_right, robot_stop

    if string_input == GUICommands.CP_USER_CONTROL.value:
        change_cp_user_control.set()
    elif string_input == GUICommands.CP_FOLLOW_LEFT_WALL.value:
        change_cp_left_wall.set()
    elif string_input == GUICommands.CP_FOLLOW_RIGHT_WALL.value:
        change_cp_right_wall.set()
    elif string_input == GUICommands.CP_FREE_ROAMING.value:
        change_cp_free_roaming.set()
    elif string_input == GUICommands.BP_FORWARD.value:
        robot_move_forward.set()
    elif string_input == GUICommands.BP_LEFT.value:
        robot_move_left.set()
    elif string_input == GUICommands.BP_BACK.value:
        robot_move_back.set()
    elif string_input == GUICommands.BP_RIGHT.value:
        robot_move_right.set()
    elif string_input == GUICommands.BP_STOP.value:
        robot_stop.set()
    elif string_input == GUICommands.BP_START.value:
        robot_start.set()

def server_listen():
    """
    server_listen is a function that runs on a separate thread and listens for inputs from the GUI on the client side

    :return: None
    """
    while robot_loop:
        message = server.receive_message()
        process_user_commands(message)

def distance_scanner():
    """
    distance_scanner is a function that runs on a separate thread and sets the control method for the servo and
        continually scans the surroundings using the distance sensor. Once the distance sensor finds a distance
        that is shorter than the set minimum distance it sets an event.

    :return: None
    """
    global robot_loop, sensor_event, servo_roaming
    while robot_loop:
        if not sensor_event.isSet() and servo_roaming.isSet():
            for num in range(int(NUM_ANGLES/2 + 1)):
                distance_mm = distance_sensor.read_mm()
                if distance_mm <= MIN_DISTANCE:
                    sensor_event.set()
                    break
                # Move the servo to designated position
                servo.rotate_servo(30 + num * 180 / NUM_ANGLES)
                time.sleep(SERVO_WAIT_TIME)
        elif not sensor_event.isSet() and servo_left.isSet():
            servo.rotate_servo(ANGLE_SLIGHT_LEFT)
            time.sleep(SERVO_WAIT_TIME)
            sensor_event.set()
            servo_left.clear()
        elif not sensor_event.isSet() and servo_right.isSet():
            servo.rotate_servo(ANGLE_SLIGHT_RIGHT)
            time.sleep(SERVO_WAIT_TIME)
            sensor_event.set()
            servo_right.clear()
        time.sleep(SHORT_WAIT_TIME)

def robot_logic():
    """
    robot_logic is a function that runs the main thread that controls the GoPiGo3 robot. This thread communicates with
        the other threads and share resources by setting and clearing events. The robot has four major control
        methods, namely: following the left wall using a PID controller, following the right wall using a PID
        controller, roaming freely and avoiding obstacles, and manual user control from the GUI client.

    :return: None
    """
    global robot_loop, robot_reset, robot_initial_set, gpg, servo, distance_sensor, current_control_method
    global sensor_event, loop_counter
    global change_cp_user_control, change_cp_left_wall, change_cp_right_wall, change_cp_free_roaming
    global robot_move_forward, robot_move_left, robot_move_back, robot_move_right, robot_stop

    found_left_wall = False
    found_right_wall = False

    while robot_loop:
        # Periodically send information to the GUI
        if loop_counter % 100 == 0:
            battery_voltage = str(gpg.volt())
            server.send_message(battery_voltage)

        # Check input commands for GUI
        if change_cp_left_wall.isSet():
            current_control_method = ControlMethod.MODE_FOLLOW_LEFT_WALL
            robot_reset = True
            change_cp_left_wall.clear()
        elif change_cp_right_wall.isSet():
            current_control_method = ControlMethod.MODE_FOLLOW_RIGHT_WALL
            robot_reset = True
            change_cp_right_wall.clear()
        elif change_cp_user_control.isSet():
            current_control_method = ControlMethod.MODE_USER_CONTROL
            robot_reset = True
            change_cp_user_control.clear()
        elif change_cp_free_roaming.isSet():
            current_control_method = ControlMethod.MODE_FREE_ROAMING
            robot_reset = True
            change_cp_free_roaming.clear()

        # Check battery_voltage and stop if robot is out of charge
        if gpg.volt() < MINIMUM_BATTERY_VOLTAGE:
            print(" Current GoPiGo battery voltage is critically low: %0.02fV\n The program will now exit... "
                  "\n Please recharge or replace the batteries before trying again..." % (gpg.volt()))
            gpg.stop()
            servo_roaming.clear()
            sys.exit(0)

        # Check if initial startup and run initialisation code
        if robot_initial_set:
            setup_bot(gpg, servo, current_control_method)
            sensor_event.clear()
            found_left_wall = False
            found_right_wall = False
            robot_initial_set = False

        # Check if robot was reset and run initialisation code
        if robot_reset:
            setup_bot(gpg, servo, current_control_method)
            robot_start.set()
            servo_roaming.set()
            sensor_event.clear()
            found_left_wall = False
            found_right_wall = False
            robot_reset = False

        # Check if robot was started by GUI
        if robot_start.isSet() and current_control_method != ControlMethod.MODE_USER_CONTROL:
            gpg.set_speed(ROBOT_SPEED)
            gpg.forward()
            servo_roaming.set()
            robot_start.clear()
        elif current_control_method == ControlMethod.MODE_USER_CONTROL:
            # Separate if condition to ensure that robot does not move spontaneously when user control is selected
            gpg.set_speed(ROBOT_SPEED)
            servo_roaming.set()
            robot_start.clear()

        # Check if robot was stopped by GUI
        if robot_stop.isSet():
            found_left_wall = False
            found_right_wall = False
            gpg.stop()
            sensor_event.clear()
            servo_roaming.clear()
            robot_stop.clear()

        # If the current control method is free roaming then the robot moves until the distance sensor is triggered,
        # then the robot stops, scans its surroundings and moves in the direction with the most open space
        if current_control_method == ControlMethod.MODE_FREE_ROAMING:
            if sensor_event.isSet():
                avoid_obstacle(gpg, distance_sensor, servo)
                time.sleep(SHORT_WAIT_TIME)
                sensor_event.clear()

        # If the current control method is to follow the left wall then the robot moves until the distance sensor is
        # triggered, then the robot stops and uses a PID controller to follow the wall
        if current_control_method == ControlMethod.MODE_FOLLOW_LEFT_WALL:
            if not found_left_wall and sensor_event.isSet():
                servo_left.set()
                servo_roaming.clear()
                found_left_wall = True
                sensor_event.clear()
                time.sleep(SHORT_WAIT_TIME)
            elif found_left_wall and sensor_event.isSet():

                distance = distance_sensor.read_mm()
                correction = pid_controller_update(distance, pid_controller, gpg, is_right=False)

                # The control system has moved out of its stable locus as contact has been lost with the wall
                if correction > THRESHOLD or correction < -THRESHOLD:
                    # Move to initial condition where the robot roams and tries to find the left wall
                    robot_reset = True
                    gpg.forward()
                    current_control_method = ControlMethod.MODE_FREE_ROAMING
                time.sleep(SHORT_WAIT_TIME)

        # If the current control method is to follow the right wall then the robot moves until the distance sensor is
        # triggered, then the robot stops and uses a PID controller to follow the wall
        if current_control_method == ControlMethod.MODE_FOLLOW_RIGHT_WALL:
            if not found_right_wall and sensor_event.isSet():
                servo_right.set()
                servo_roaming.clear()
                found_right_wall = True
                sensor_event.clear()
                time.sleep(SHORT_WAIT_TIME)
            elif found_right_wall and sensor_event.isSet():
                distance = distance_sensor.read_mm()
                correction = pid_controller_update(distance, pid_controller, gpg, is_right=True)

                # The control system has moved out of its stable locus as contact has been lost with the wall
                if correction > THRESHOLD or correction < -THRESHOLD:
                    # Move to initial condition where the robot roams and tries to find the right wall
                    robot_reset = True
                    gpg.forward()
                    current_control_method = ControlMethod.MODE_FREE_ROAMING
                time.sleep(SHORT_WAIT_TIME)

        # If the current control method is to user input then the robot moves as instructed by the user
        elif current_control_method == ControlMethod.MODE_USER_CONTROL:
            if robot_move_left.isSet():
                gpg.stop()
                gpg.turn_degrees(-TURN_DEGREES)
                time.sleep(SHORT_WAIT_TIME)
                robot_move_left.clear()
            elif robot_move_right.isSet():
                gpg.stop()
                gpg.turn_degrees(TURN_DEGREES)
                time.sleep(SHORT_WAIT_TIME)
                robot_move_right.clear()
            elif robot_move_forward.isSet():
                gpg.forward()
                time.sleep(MOVE_TIME)
                gpg.stop()
                robot_move_forward.clear()
            elif robot_move_back.isSet():
                gpg.backward()
                time.sleep(MOVE_TIME)
                gpg.stop()
                robot_move_back.clear()

        time.sleep(SHORT_WAIT_TIME)
        loop_counter+=1

if __name__ == '__main__':
    # global variables that
    loop_counter = 0
    robot_loop = True
    robot_reset = False
    robot_initial_set = True

    # events to communicate between the threads
    sensor_event = Event()
    servo_roaming = Event()
    servo_right = Event()
    servo_left = Event()

    robot_move_left = Event()
    robot_move_right = Event()
    robot_move_forward = Event()
    robot_move_back = Event()
    robot_stop = Event()
    robot_start = Event()

    change_cp_user_control = Event()
    change_cp_left_wall = Event()
    change_cp_right_wall = Event()
    change_cp_free_roaming = Event()

    # three different threads, one for the main loop, one for the distance scanner and one for the running the server
    robot_main_thread = Thread(target=robot_logic, name="1")
    distance_scanner_thread = Thread(target=distance_scanner, name="2")
    server_listen_thread = Thread(target=server_listen, name="3")

    distance_scanner_thread.start()
    robot_main_thread.start()
    server_listen_thread.start()