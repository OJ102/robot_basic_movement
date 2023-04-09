#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, ColorSensor
# from pybricks.ev3devices import GyroSensor
# from pybricks.ev3devices import MediumMotor
from pybricks.tools import DataLog, StopWatch, wait
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the gyro sensor
proxi_sensor=UltrasonicSensor(Port.S1)
gyro=GyroSensor(Port.S4)
color_sensor=ColorSensor(Port.S2)

lift_motor = Motor(Port.A)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=110)


def turn(a):
    s = 200 # mm/s

    while gyro.angle() != a:
        left_motor.run(speed=s)
        right_motor.run(speed=(-1 * s))
        print(gyro.angle())
    left_motor.brake()
    right_motor.brake()
    ev3.speaker.beep()    

def straight():
    # Set the desired angle for going straight
    target_angle = 0
    gyro.reset_angle(0)
    # Set the motor power for going straight
    motor_power = 100

    while True:
        # Calculate the current angle of the robot using the gyro sensor
        angle = gyro.angle()
        left_motor.run(speed=500)
        right_motor.run(speed=500)
        # Calculate the error between the current angle and the desired angle
        error = angle - target_angle


        # Calculate the correction factor based on the error
        correction = error * 1.5
        print(gyro.angle())
        # Stop the loop when the robot is within a certain tolerance of the target angle
        if abs(angle)>5:
            turn(0)
            
        
    # Stop the motors
    robot.stop()
straight()