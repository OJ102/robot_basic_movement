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
v_dps=-500 # Velocity in degs per second

# Callibrated distn to make bot actual go entered distance
distn=distn_cm*14

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
lift_motor = Motor(Port.A)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
proxi_sensor = UltrasonicSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S4)


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=43.8, axle_track=160)

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


def turn():
    angle = 90 # degrees
    speed = 200 # mm/s

    gyro_sensor.reset_angle(0)
    while gyro_sensor.angle() < angle:
        left_motor.run(speed=speed)
        right_motor.run(speed=(-1 * speed))

    ev3.speaker.beep() 

    left_motor.brake()
    right_motor.brake()    

# turn()

def lift():
    lift_motor.run(speed=-500)
    wait(500)
    lift_motor.stop()

# lift()

def proxi():
    while True:
        left_motor.run(speed=500)
        right_motor.run(speed=500)
        if proxi_sensor.distance() < 300:
            break
proxi()