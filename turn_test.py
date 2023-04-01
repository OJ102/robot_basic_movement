#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
# from pybricks.ev3devices import GyroSensor
# from pybricks.ev3devices import MediumMotor
from pybricks.tools import wait
from pybricks.parameters import Port
from pybricks.robotics import DriveBase


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

lift_motor = Motor(Port.A)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=43.8, axle_track=160)

# Setting the speed for the motors
robot.settings(v_dps)

def turn():
    angle = 180 # degrees
    speed = 200 # mm/s

    gyro_sensor.reset_angle(0)
    while gyro_sensor.angle() < angle:
        left_motor.run(speed=speed)
        right_motor.run(speed=(-1 * speed))

    ev3.speaker.beep() 

    left_motor.brake()
    right_motor.brake()    

turn()
