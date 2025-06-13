""" Code for lower half of track, exiting left """

from machine import I2C, Pin, ADC
from time import sleep
from motor import Motor
from servo import setServoAngle
from ultrasonic import sonic
from APDS9960LITE import APDS9960LITE


""" Functions """

def exit_left():
    """ Leave the garage and turn left """
    # Drive forward
    for i in range(0, 24):
        motor_left.ctrl_alloc(1, 50)
        motor_right.ctrl_alloc(1, 50)
        sleep(0.025)

        motor_left.ctrl_alloc(0, 0)
        motor_right.ctrl_alloc(0, 0)
        sleep(0.025)

    # Turn left ~90 degrees
    for i in range(0, 8):
        motor_left.ctrl_alloc(1, 0)
        motor_right.ctrl_alloc(1, 60)
        sleep(0.025)

        motor_left.ctrl_alloc(1, 0)
        motor_right.ctrl_alloc(1, 0)
        sleep(0.025)

    # Drive forward
    for i in range(0, 8):
        motor_left.ctrl_alloc(1, 50)
        motor_right.ctrl_alloc(1, 50)
        sleep(0.025)

        motor_left.ctrl_alloc(0, 0)
        motor_right.ctrl_alloc(0, 0)
        sleep(0.025)

    # Stop motors
    motor_left.ctrl_alloc(0, 0)
    motor_right.ctrl_alloc(0, 0)
    sleep(0.05)


def line():
    """ Collect IR sensor readings, decide if line is detected and calculator distance from line """

    for i in range(len(A0_array)):
        A0_array[i] = A0.read_u16()
        A1_array[i] = A1.read_u16()
        A2_array[i] = A2.read_u16()
        sleep(0.0001)
    A0_val = sum(A0_array) / len(A0_array)
    A1_val = sum(A1_array) / len(A1_array)
    A2_val = sum(A2_array) / len(A2_array)

    A = [A0_val, A1_val, A2_val]

    # Check if any ADC sensor detects a line
    if A[0] >= line_threshold or A[1] >= line_threshold or A[2] >= line_threshold:
        line_detected = True
    else:
        line_detected = False

    # Estimate line position if line is detected
    if line_detected:
        num = A[0] * x[0] + A[1] * x[1] + A[2] * x[2]
        denom = A[0] + A[1] + A[2]
        if denom != 0:
            line_dist = num / denom
        else:
            line_dist = 0
    else:
        line_dist = "Undefined"

    return A, line_detected, line_dist


def US_forward():
    """ Take US sensor reading in front of vehicle """

    for i in range(len(US_array)):
        US_array[i] = US_sensor.distance_mm()
        sleep(0.002)
    US_fwd = sum(US_array) / len(US_array)

    return US_fwd


def US_scan():
    """ Take US sensor readings in front and to each side of vehicle """

    # Stop motors
    motor_left.ctrl_alloc(0, 0)
    motor_right.ctrl_alloc(0, 0)

    # Scan left
    setServoAngle(angle_left)
    sleep(0.3)
    for i in range(len(US_array)):
        US_array[i] = US_sensor.distance_mm()
        sleep(0.01)
    left_dist = sum(US_array) / len(US_array)

    # Scan forward
    setServoAngle(angle_fwd)
    sleep(0.3)
    for i in range(len(US_array)):
        US_array[i] = US_sensor.distance_mm()
        sleep(0.01)
    fwd_dist = sum(US_array) / len(US_array)

    # Scan right
    setServoAngle(angle_right)
    sleep(0.3)
    for i in range(len(US_array)):
        US_array[i] = US_sensor.distance_mm()
        sleep(0.01)
    right_dist = sum(US_array) / len(US_array)

    setServoAngle(angle_fwd)

    print(f'Distances - forward: {fwd_dist}, left: {left_dist}, right: {right_dist}')

    return fwd_dist, left_dist, right_dist


def roundabout_check():
    """ Inch forward and check if IR sensors still detect a line"""

    # Drive forward
    for i in range(0, 8):
        motor_left.ctrl_alloc(1, 50)
        motor_right.ctrl_alloc(1, 50)
        sleep(0.025)

        motor_left.ctrl_alloc(0, 0)
        motor_right.ctrl_alloc(0, 0)
        sleep(0.025)

    # Stop motors
    motor_left.ctrl_alloc(0, 0)
    motor_right.ctrl_alloc(0, 0)
    sleep(0.05)

    return [A0.read_u16(), A1.read_u16(), A2.read_u16()]


def roundabout_right():
    """ Take first exit on the right """

    # Drive forward
    for i in range(0, 8):
        motor_left.ctrl_alloc(1, 50)
        motor_right.ctrl_alloc(1, 50)
        sleep(0.025)

        motor_left.ctrl_alloc(1, 0)
        motor_right.ctrl_alloc(1, 0)
        sleep(0.025)

    # Turn left ~90 degrees
    for i in range(0, 12):
        motor_left.ctrl_alloc(1, 60)
        motor_right.ctrl_alloc(1, 0)
        sleep(0.025)

        motor_left.ctrl_alloc(1, 0)
        motor_right.ctrl_alloc(1, 0)
        sleep(0.025)

    # Drive forward
    for i in range(0, 8):
        motor_left.ctrl_alloc(1, 50)
        motor_right.ctrl_alloc(1, 50)
        sleep(0.025)

        motor_left.ctrl_alloc(1, 0)
        motor_right.ctrl_alloc(1, 0)
        sleep(0.025)

    # Stop motors
    motor_left.ctrl_alloc(0, 0)
    motor_right.ctrl_alloc(0, 0)
    sleep(0.05)


def dead_end():
    """ Back away from wall then turn anticlockwise to reverse direction of vehicle """

    # Drive backward
    motor_left.ctrl_alloc(0, 50)
    motor_right.ctrl_alloc(0, 50)
    sleep(0.2)

    # Spin around anticlockwise
    motor_left.ctrl_alloc(0, 50)
    motor_right.ctrl_alloc(1, 50)
    sleep(0.5)

    # Stop motors
    motor_left.ctrl_alloc(0, 0)
    motor_right.ctrl_alloc(0, 0)
    sleep(0.05)


""" Initialise components and variables """

# Sensor pins
ADC0 = 26
ADC1 = 27
ADC2 = 28
ECHO = 2
TRIG = 3

# Initialize motors
motor_right  = Motor("right", 9, 8, 6)
motor_left = Motor("left", 11, 10, 7)

# Initialize line sensors
A0 = ADC(Pin(ADC0))
A1 = ADC(Pin(ADC1))
A2 = ADC(Pin(ADC2))

# Distances from the centroid of the robot to the centre of each sensor in mm
x = [-15, 0, 15]

# Initialize ultrasonic sensor
US_sensor = sonic(TRIG, ECHO)
angle_fwd = 85
angle_left = 170
angle_right = 15

# Initialise RGB sensor
i2c = I2C(0, scl=Pin(17), sda=Pin(16))
apds9960=APDS9960LITE(i2c)      # Create APDS9660 sensor object
apds9960.als.enableSensor()     # Send I2C command to enable RGB sensor
apds9960.als.eLightGain=3       # Set RGB sensor gain

# Motor control variables
dir_left = 1
dir_right = 1
pwm_left = 0
pwm_right = 0

line_threshold = 3000

US_threshold_fwd = 90
US_threshold_park = 60
US_threshold_side = 90

upper_light_threshold = 180
lower_light_threshold = 120

base_speed_left = 55
base_speed_right = 55

line_dist_p1 = 0
line_dist_p2 = 0

line_detected_p1 = False
line_detected_p2 = False

A_p1 = [0, 0, 0]
A_p2 = [0, 0, 0]

A0_array = [0, 0, 0, 0, 0]
A1_array = [0, 0, 0, 0, 0]
A2_array = [0, 0, 0, 0, 0]

US_array = [0, 0, 0]

Amb_array = [0, 0, 0, 0, 0]
R_array = [0, 0, 0, 0, 0]
G_array = [0, 0, 0, 0, 0]
B_array = [0, 0, 0, 0, 0]

S = 0
P = 0

# Reset servo angle
sleep(0.02)
setServoAngle(angle_fwd)


""" Exit garage """

sleep(3)
exit_left()


""" State machine """

# Declare states
state_list = ['FOLLOW_LINE', 'SEARCHING_LEFT', 'SEARCHING_RIGHT', 'ROUNDABOUT_CHECK', 'ROUNDABOUT', 'WALL_CHECK', 'DEAD_END', 'GARAGE', 'PARKED']
state = 'SEARCHING_RIGHT'  # Initial state

while True:

    """ Initial section of state machine loop """

    # Collect IR sensor data
    A, line_detected, line_dist = line()

    # Collect forward US sensor data
    US_fwd = US_forward()

    print(f"State: {state}")
    print(f"ADC readings: {A[0]}, {A[1]}, {A[2]}, Line dist.: {line_dist}")
    print(f"US reading: {US_fwd}")


    """ States """

    # Make decisions based on current state
    if state == 'FOLLOW_LINE':

        if line_dist != "Undefined":
            # Calculate course correction to keep vehicle centered on line
            # Positive correction: line is right of center, steer right
            # Negative correction: line is left of center, steer left
            line_correction_gain = 2    # tune value
            line_correction = line_dist * line_correction_gain

            dir_left = 1
            dir_right = 1
            # Adjust motors based on calculated correction
            pwm_left = (base_speed_left + line_correction) // 1
            pwm_right = (base_speed_right - line_correction) // 1

        elif line_dist == 'Undefined' and line_dist_p1 != 'Undefined':
            if line_dist_p1 >= 0:
                state = 'SEARCHING_RIGHT'
            else:
                state = 'SEARCHING_LEFT'

        if A[0] >= line_threshold and A[1] >= line_threshold and A[2] >= line_threshold:
            state = 'ROUNDABOUT_CHECK'

        if 0 < US_fwd <= US_threshold_fwd:
            state = 'WALL_CHECK'


    elif state == 'SEARCHING_LEFT':

        # Veer left
        dir_left = 1
        pwm_left = 40
        dir_right = 1
        pwm_right = 75

        if A[0] >= line_threshold or A[1] >= line_threshold or A[2] >= line_threshold:
            state = 'FOLLOW_LINE'

        if 0 < US_fwd <= US_threshold_fwd:
            state = 'WALL_CHECK'

        elif S <= 10:
            S += 1

            state = state

        else:
            S = 0

            state = 'SEARCHING_RIGHT'


    elif state == 'SEARCHING_RIGHT':

        # Veer right
        dir_left = 1
        pwm_left = 75
        dir_right = 1
        pwm_right = 40

        if A[0] >= line_threshold or A[1] >= line_threshold or A[2] >= line_threshold:
            state = 'FOLLOW_LINE'

        if 0 < US_fwd <= US_threshold_fwd:
            state = 'WALL_CHECK'

        elif S <= 10:
            S += 1

            state = state

        else:
            S = 0

            state = 'SEARCHING_LEFT'


    elif state == 'ROUNDABOUT_CHECK':

        # Inch forward and take IR readings
        A = roundabout_check()

        # Transition conditions
        if A[0] >= line_threshold and A[1] >= line_threshold and A[2] >= line_threshold:
            state = 'ROUNDABOUT_CHECK'

        elif A[0] < line_threshold and A[1] < line_threshold and A[2] < line_threshold:
            state = 'ROUNDABOUT'

        else:
            state = 'FOLLOW_LINE'


    elif state == 'ROUNDABOUT':

        # Take first exit on the right
        roundabout_right()

        dir_left = 0
        pwm_left = 0
        dir_right = 0
        pwm_right = 0

        state = 'FOLLOW_LINE'

        if 0 < US_fwd <= US_threshold_fwd:
            state = 'WALL_CHECK'


    elif state == 'WALL_CHECK':

        # Collect US sensor data in front and to each side
        US_fwd, US_L, US_R = US_scan()

        dir_left = 0
        dir_right = 0
        pwm_left = 0
        pwm_right = 0

        if 0 < US_fwd <= US_threshold_fwd:

            if 0 < US_L <= US_threshold_side and 0 < US_R <= US_threshold_side:
                state = 'GARAGE'

            else:
                state = 'DEAD_END'

        else:
            state = 'FOLLOW_LINE'


    elif state == 'DEAD_END':

        # Back away and turn around
        dead_end()

        dir_left = 0
        dir_right = 0
        pwm_left = 0
        pwm_right = 0

        state = 'SEARCHING_LEFT'


    elif state == 'GARAGE':

        # Collect US sensor data in front and to each side
        US_fwd, US_L, US_R = US_scan()

        dir_left = 0
        dir_right = 0
        pwm_left = 0
        pwm_right = 0

        if 0 < US_fwd <= US_threshold_park:
            state = 'PARKED'

        else:
            if 0 < US_L <= US_threshold_side and 0 < US_R <= US_threshold_side:
                wall_pos = US_L - US_R
                wall_gain = 0.4
                wall_correct = wall_pos * wall_gain

                dir_left = 1
                dir_right = 1
                pwm_left = (base_speed_left - wall_correct) // 1
                pwm_right = (base_speed_right + wall_correct) // 1

            else:
                state = 'WALL_CHECK'


    elif state == 'PARKED':

        if P >= 5:
            dir_left = 0
            pwm_left = 0
            dir_right = 0
            pwm_right = 0

        elif 0 < US_fwd < US_threshold_park:
            dir_left = 0
            pwm_left = 0
            dir_right = 0
            pwm_right = 0
            P += 1

        else:
            P = 0
            state = 'GARAGE'


    """ Final section of state machine loop """

    # Convert PWM values to integers
    pwm_left = int(pwm_left)
    pwm_right = int(pwm_right)

    print("left {} @ {}".format(dir_left, pwm_left))
    print("right {} @ {}".format(dir_right, pwm_right))

    # Send control signals to motors
    motor_left.ctrl_alloc(dir_left, pwm_left)
    motor_right.ctrl_alloc(dir_right, pwm_right)
    sleep(0.040)

    motor_left.ctrl_alloc(0, 0)
    motor_right.ctrl_alloc(0, 0)
    sleep(0.015)

    # Keep readings from previous two loops
    line_dist_p2 = line_dist_p1
    line_dist_p1 = line_dist
    A_p2 = A_p1
    A_p1 = A