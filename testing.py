#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor
# from pybricks.ev3devices import GyroSensor
# from pybricks.ev3devices import MediumMotor
from pybricks.tools import DataLog, StopWatch, wait
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import os

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
robot = DriveBase(left_motor, right_motor, wheel_diameter=43.8, axle_track=160)

# Start a stopwatch to measure elapsed time
watch = StopWatch()
def t1():
    distn=1000
    rd=0
    while True:
        # Resets the time to 0
        watch.reset()

        # Begin driving forward at 200 millimeters per second.
        left_motor.run(speed=500)
        right_motor.run(speed=500)

        # Wait until an obstacle is detected. This is done by repeatedly
        # doing nothing (waiting for 10 milliseconds) while the measured
        # distance is still greater than 400 mm.
        if proxi_sensor.distance() < 300:
            
            ev3.speaker.beep() 

            left_motor.brake()
            right_motor.brake()

            wait(1000)
        # Checks if the robot has reached the required distance
        if rd>=distn:
            break
        rd+=watch.time()*500
        print(rd)
        
t1()