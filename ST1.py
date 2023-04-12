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
robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=110)

# Start a stopwatch to measure elapsed time
watch = StopWatch()

# Setting the speed for the motors
robot.settings(v_dps)

def start():
    for i in range(10):
        ev3.speaker.beep()
start()

def inch_to_mm(inch):
    mm=inch*25.4
    return mm

def box_dist(box_num):
    box_d_inches=(6*(box_num-6))
    return box_d_inches

def turn(a,sp):
    angle = a # degrees
    s = sp # mm/s
    
    #callibrates sensor
    robot.reset() 
    gyro_sensor.reset_angle(0)

    while gyro_sensor.angle() <= angle:
        left_motor.run(speed=s)
        right_motor.run(speed=(-1 * s))
        
    left_motor.brake()
    right_motor.brake()
    ev3.speaker.beep()     

def proxi(s):

    while True:
        left_motor.run(speed=s)
        right_motor.run(speed=s)
        if proxi_sensor.distance() < 200:
            left_motor.brake()
            right_motor.brake()
            break

    lift(-500)

    while True:
        left_motor.run(speed=-500)
        right_motor.run(speed=-500)
        wait(1000)
        left_motor.brake()
        right_motor.brake()
        break
    
    lift(500)


# proxi()

def obsavoider(d,s):
    distn=d
    robot.reset()
    while True:
        # Begin driving forward at 200 millimeters per second.
        left_motor.run(speed=s)
        right_motor.run(speed=s)
        # Wait until an obstacle is detected. This is done by repeatedly
        # doing nothing (waiting for 10 milliseconds) while the measured
        # distance is still greater than 400 mm.
        if proxi_sensor.distance() < 300:
            
            ev3.speaker.beep() 

            left_motor.brake()
            right_motor.brake()
            
            wait(2000)
        # Checks if the robot has reached the required distance
        if robot.distance()>=distn:
            left_motor.brake()
            right_motor.brake()
            break
        print(robot.distance())

def gyro_fwd(d,s):
    distn=d
    print("gyrofwd")
    robot.reset()
    gyro_sensor.reset_angle(0)
    while True:

        left_motor.run(speed=s)
        right_motor.run(speed=s)
        print(gyro_sensor.angle())
        while gyro_sensor.angle() < -5:
            left_motor.run(speed=s)
            right_motor.run(speed=(-1 * s))
            if gyro_sensor.angle()==0:
                break
            print(gyro_sensor.angle())
        while gyro_sensor.angle() > 5:
            left_motor.run(speed=-1*s)
            right_motor.run(speed=(s))
            print(gyro_sensor.angle())
            if gyro_sensor.angle()==0:
                break
            print(gyro_sensor.angle())
        if robot.distance()>=distn:
            left_motor.brake()
            right_motor.brake()
            break
        print(robot.distance())

def main_ST1():
    box=11#put the number of the box here
    x=inch_to_mm(box_dist(box))
    y=inch_to_mm(30+2)

    x_return=inch_to_mm(96-box_dist(box)+3)
    print(x,y,x_return)
    print()
    wait(5000)

    obsavoider(y,250)
    turn(85,100)
    wait(2000)
    obsavoider(x,250)
    ev3.speaker.beep()
    wait(5000)

    obsavoider(x_return,250)
    turn(85,100)
    wait(2000)
    obsavoider(y,250)

main_ST1()
