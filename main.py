'''
Run python Single_ODrive_MoveRobot.py -mode calibration  for calibration
RUn python Single_ODrive_MoveRobot.py					 for no calibration
'''

# motor is AMT102
from __future__ import print_function, division

import odrive
from odrive.enums import *
import time
import math
import argparse
from tkinter import *
from pynput import keyboard
import numpy as np

class Errors:
    axis = {
        0x00: "ERROR_NONE",
        0x01: "ERROR_INVALID_STATE",
        0x02: "ERROR_DC_BUS_UNDER_VOLTAGE",
        0x04: "ERROR_DC_BUS_OVER_VOLTAGE",
        0x08: "ERROR_CURRENT_MEASUREMENT_TIMEOUT",
        0x10: "ERROR_BRAKE_RESISTOR_DISARMED",
        0x20: "ERROR_MOTOR_DISARMED",
        0x40: "ERROR_MOTOR_FAILED",
        0x80: "ERROR_SENSORLESS_ESTIMATOR_FAILED",
        0x100: "ERROR_ENCODER_FAILED",
        0x200: "ERROR_CONTROLLER_FAILED",
        0x400: "ERROR_POS_CTRL_DURING_SENSORLESS",
        0x800:

        "ERROR_WATCHDOG_TIMER_EXPIRED"
    }

    axis_states = {
        0: "AXIS_STATE_UNDEFINED",
        1: "AXIS_STATE_IDLE",
        2: "AXIS_STATE_STARTUP_SEQUENCE",
        3: "AXIS_STATE_FULL_CALIBRATION_SEQUENCE",
        4: "AXIS_STATE_MOTOR_CALIBRATION",
        5: "AXIS_STATE_SENSORLESS_CONTROL",
        6: "AXIS_STATE_ENCODER_INDEX_SEARCH",
        7: "AXIS_STATE_ENCODER_OFFSET_CALIBRATION",
        8: "AXIS_STATE_CLOSED_LOOP_CONTROL",
        9: "AXIS_STATE_LOCKIN_SPIN",
        10: "AXIS_STATE_ENCODER_DIR_FIND"
    }


    motor = {
        0: "ERROR_NONE",
        0x0001: "ERROR_PHASE_RESISTANCE_OUT_OF_RANGE",
        0x0002: "ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE",
        0x0004: "ERROR_ADC_FAILED",
        0x0008: "ERROR_DRV_FAULT",
        0x0010: "ERROR_CONTROL_DEADLINE_MISSED",
        0x0020: "ERROR_NOT_IMPLEMENTED_MOTOR_TYPE",
        0x0040: "ERROR_BRAKE_CURRENT_OUT_OF_RANGE",
        0x0080: "ERROR_MODULATION_MAGNITUDE",
        0x0100: "ERROR_BRAKE_DEADTIME_VIOLATION",
        0x0200: "ERROR_UNEXPECTED_TIMER_CALLBACK",
        0x0400: "ERROR_CURRENT_SENSE_SATURATION",
        0x1000: "ERROR_CURRENT_UNSTABLE"
    }

    encoder = {
        0: "ERROR_NONE",
        0x01: "ERROR_UNSTABLE_GAIN",
        0x02: "ERROR_CPR_OUT_OF_RANGE",
        0x04: "ERROR_NO_RESPONSE",
        0x08: "ERROR_UNSUPPORTED_ENCODER_MODE",
        0x10: "ERROR_ILLEGAL_HALL_STATE",
        0x20: "ERROR_INDEX_NOT_FOUND_YET"
    }

    controller = {
        0: "ERROR_NONE",
        0x01: "ERROR_OVERSPEED",
    }


    motor_types = {
        0: "MOTOR_TYPE_HIGH_CURRENT",
        1: "MOTOR_TYPE_LOW_CURRENT",
        2: "MOTOR_TYPE_GIMBAL"
    }

    control_modes = {
        0: "CTRL_MODE_VOLTAGE_CONTROL",
        1: "CTRL_MODE_CURRENT_CONTROL",
        2: "CTRL_MODE_VELOCITY_CONTROL",
        3: "CTRL_MODE_POSITION_CONTROL",
        4: "CTRL_MODE_TRAJECTORY_CONTROL"
    }

    encoder_modes = {
        0: "ENCODER_MODE_INCREMENTAL",
        1: "ENCODER_MODE_HALL"
    }


testing = False  # testing variable
command = ["Stopped", 0]
increment = 0.5

config = {
    "wheel_radius": 0.1,
    "drive_gearing": 16,  # 64 means a gearing of 64 to one
    "flipper_gearing": 5
}


def on_press(key):
    try:
        character = key.char

        if character == "a":
            command[0] = "Left"
        elif character == "w":
            command[0] = "Forward"
        elif character == "s":
            command[0] = "Reverse"
        elif character == "d":
            command[0] = "Right"

    except AttributeError:
        if key == keyboard.Key.up:
            command[1] += increment
        elif key == keyboard.Key.down:
            command[1] -= increment


def on_release(key):
    try:
        key.char
        command[0] = "Stopped"

    except AttributeError:
        return 0


def parse_args():
    parser = argparse.ArgumentParser('Settings for ODrive')

    parser.add_argument(
        '-mode', default='no calibration', help='parsing in calibration will do the full reset and calibrate stage'
    )

    return parser.parse_args()


def determineSpeed():
    max = 300
    state = command[0]
    absSpeed = round(command[1], 2)
    if absSpeed > max:
        absSpeed = max

    speedSetting = [absSpeed, absSpeed]
    realSpeed = [0, 0]

    if state == "Right":
        speedSetting = [-1 * absSpeed, absSpeed]
    elif state == "Forward":
        speedSetting = [absSpeed, absSpeed]
    elif state == "Reverse":
        speedSetting = [-1 * absSpeed, -1 * absSpeed]
    elif state == "Left":
        speedSetting = [absSpeed, -1 * absSpeed]

    if state != "Stopped":
        realSpeed = speedSetting

    return speedSetting, realSpeed


#### ALL ODRIVE COMMANDS FROM HERE ####
def full_reset_and_calibrate(odrv0):
    """Completely resets the Odrive, calibrates axis0 and configures axis0 to only encoder index search on startup and be ready in AXIS_STATE_CLOSED_LOOP_CONTROL"""
    odrv0.erase_configuration()
    print("Erased [1/7]")
    try:  # Reboot causes loss of connection, use try to supress errors
        odrv0.reboot()
    except:
        pass
    print("Rebooted [2/7]")
    odrv0 = odrive.find_any()  # Reconnect to the Odrive
    print("Connected [3/7]")
    odrv0.axis0.motor.config.pre_calibrated = True  # Set all the flags required for pre calibration
    odrv0.axis0.encoder.config.pre_calibrated = True
    odrv0.axis0.encoder.config.use_index = True
    odrv0.axis0.config.startup_encoder_index_search = True  # Change startup sequence
    odrv0.axis0.config.startup_closed_loop_control = True
    odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE  # Calibrate
    print("Started calibration 1 [4/7]", end="")
    while odrv0.axis0.current_state != AXIS_STATE_IDLE:  # Wait for calibration to be done
        time.sleep(0.1)
        print(".", end="")
    odrv0.save_configuration()

    print("\nCalibration 1 complete [5/7]")
    print("now will begin calibration sequence for second axis")
    time.sleep(3)
    odrv0.axis1.motor.config.pre_calibrated = True  # Set all the flags required for pre calibration
    odrv0.axis1.encoder.config.pre_calibrated = True
    odrv0.axis1.encoder.config.use_index = True
    odrv0.axis1.config.startup_encoder_index_search = True  # Change startup sequence
    odrv0.axis1.config.startup_closed_loop_control = True
    odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE  # Calibrate
    print("Started calibration 2 [6/7]", end="")
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:  # Wait for calibration to be done
        time.sleep(0.5)
        print(".", end="")

    print("\nCalibration 2 complete [7/7]")

    # closed loop control for both axis
    odrv0.save_configuration()
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    return odrv0


def set_rps(axis, rps):
    '''
    et axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL.
    Set the velocity ramp rate (acceleration): axis.controller.config.vel_ramp_rate = 2000 [counts/s^2]
    Activate the ramped velocity mode: axis.controller.vel_ramp_enable = True.
    You can now control the velocity with axis.controller.vel_ramp_target = 5000 [count/s].
    '''
    axis.controller.config.control_mode = 2

    axis.controller.vel_setpoint = rps * 8192


def set_velocity(axis, v):
    rps = v * config["drive_gearing"] / (2 * 3.1415 * config["wheel_radius"])
    # print(rps * 8192)
    set_rps(axis, rps)
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


args = parse_args()
# check if the setting parsed is correct
if args.mode != 'calibration':
    print("Will begin process without calibration")
else:
    print("Execute full reset and calibrate sequence")

print("\nDo you want to continue? Press enter if so")
input()

app = Tk()
app.title("Velocity control")
app.geometry("500x200")
Direction = Label(app, text="Direction = Stopped")
Direction.pack(side=TOP)
CurrentSpeed = Label(app, text="Current speed = left: 0  right: 0")
CurrentSpeed.pack(side=TOP)
Speed = Label(app, text="Speed setting = left: 0  right: 0")
Speed.pack(side=TOP)

if args.mode != 'control_test':
    # Find a connected ODrive (this will block until you connect one)
    print("Finding an odrive...")
    my_drive = odrive.find_any()

    if args.mode == 'calibration':
        my_drive = full_reset_and_calibrate(my_drive)
    # To read a value, simply read the property
    # print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

    my_drive.axis0.controller.config.vel_limit = 500000.0
    my_drive.axis0.motor.config.current_lim = 8

    my_drive.axis1.controller.config.vel_limit = 500000.0
    my_drive.axis1.motor.config.current_lim = 8

else:  # if mode == control test
    testing = True

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()


def determine_sign(prevSign, currentValue):
    output = prevSign

    for i, value in enumerate(currentValue):
        if np.sign(value) != 0:
            output[i] = np.sign(value)

    return output


# control sequence
previous = 0

threshold = 10
sign = [0, 0]

while 1:
    speed = determineSpeed()
    Direction.config(text="Direction = " + command[0])
    CurrentSpeed.config(text="Current speed = left: " + str(speed[1][0]) + "right: " + str(speed[1][1]))
    Speed.config(text="Spped setting = left: " + str(speed[0][0]) + "right: " + str(speed[0][1]))
    app.update()

    maxrps = 0.5
    if not testing:

        current = speed[1][0]

        if abs(previous - current) > threshold:
            current = previous + np.sign(current - previous) * threshold

        set_velocity(my_drive.axis0, sign[0] * abs(current) * maxrps * 0.01)
        set_velocity(my_drive.axis1, sign[1] * abs(current) * maxrps * 0.01)

        previous = current
        sign = determine_sign(sign, speed[1])
        print(current)

# run some control code to test odrives
# do ros tutorials
# communicate with ros to run odrives