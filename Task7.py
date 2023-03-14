#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

#Take all values
turn_degs=90
distn_cm=100
distn_inches=12
v_dps=-500 # Velocity in degs per second


# Callibrated distn to make bot actual go entered distance
distn=(distn_inches*2.54)*14 #2.54 is to convert the inches to cm so code runs properly

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=43.8, axle_track=160)

# Setting the speed for the motors
robot.settings(v_dps)


def fwd(distn):
    # Go forward and backwards for one meter.
    robot.straight(distn)
    ev3.speaker.beep()

    # robot.straight(-distn)#reverse
    # ev3.speaker.beep()

def turn(angle):
    # Make a turn.
    robot.turn(angle)
    ev3.speaker.beep()


fwd(distn)
turn(turn_degs)

distn_inches=48
distn=(distn_inches*2.54)*14 #2.54 is to convert the inches to cm so code runs properly

fwd(distn)