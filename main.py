#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor
# from pybricks.ev3devices import GyroSensor
# from pybricks.ev3devices import MediumMotor
from pybricks.tools import DataLog, StopWatch, wait
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import testing
import ST2

#Take all values
turn_degs=180
distn_cm=100
v_dps=500 # Velocity in degs per second
x=0
y=0

# Callibrated distn to make bot actual go entered distance
distn=distn_cm*14

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
proxi_sensor=UltrasonicSensor(Port.S1)
gyro_sensor=GyroSensor(Port.S4)
color_sensor=ColorSensor(Port.S2)

lift_motor = Motor(Port.A)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=110)

# Start a stopwatch to measure elapsed time
watch = StopWatch()

# Setting the speed for the motors
robot.settings(v_dps)

def subtaska(distn):
    # Go forward and backwards for one meter.
    robot.straight(distn)
    ev3.speaker.beep()

    robot.straight(-distn)#reverse
    ev3.speaker.beep()

def subtaskb(distn,angle):
    # Go forward 'distn' meter.
    robot.straight(distn)
    ev3.speaker.beep()

    # Make a U-turn.
    robot.turn(angle)
    ev3.speaker.beep()

    # Go back to the original starting spot
    robot.straight(distn)
    ev3.speaker.beep()

# subtaska(distn)

# subtaskb(distn,turn_degs)


def start():
    for i in range(10):
        ev3.speaker.beep()
start()

def inch_to_mm(inch):
    mm=inch*25.4
    return mm


def gyro_fwd():
    print("gyrofwd")
    robot.reset()
    gyro.reset_angle(0)
    print(gyro_sensor.angle())
    while gyro_sensor.angle() <= 0:
        left_motor.run(speed=s)
        right_motor.run(speed=(-1 * s))
        print(gyro_sensor.angle())
    while gyro_sensor.angle() <= 0:
        left_motor.run(speed=-1*s)
        right_motor.run(speed=(s))
        print(gyro_sensor.angle())
gyro_fwd()