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


def turn(a):
    angle = a # degrees
    s = 200 # mm/s
    
    #callibrates sensor
    robot.reset() 
    gyro_sensor.reset_angle(0)

    while gyro_sensor.angle() < angle:
        left_motor.run(speed=s)
        right_motor.run(speed=(-1 * s))

    left_motor.brake()
    right_motor.brake()
    ev3.speaker.beep()     

# turn(180)

def lift(Speed):
    ev3.speaker.beep() 
    wait(500)
    lift_motor.run(speed=Speed)
    wait(500)
    # lift_motor.brake()

# lift(-500)

def proxi():
    while True:
        left_motor.run(speed=500)
        right_motor.run(speed=500)
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

# def navigator(x,y):
#     pos=[0,0]
#     i=0
#     j=0
#     while True:
def test():
    while True:
    # Begin driving forward at 200 millimeters per second.
        left_motor.run(speed=500)
        right_motor.run(speed=500)
    # Wait until an obstacle is detected. This is done by repeatedly
    # doing nothing (waiting for 10 milliseconds) while the measured
    # distance is still greater than 300 mm.
        if proxi_sensor.distance() < 300:
            ev3.speaker.beep() 
            # Turn around by 90 degrees
            turn(90)

test()

def gyrostrt():
    distance = 1000 # millimetres
    robotSpeed = 150 # mm/sec    

    robot.reset() 
    gyro_sensor.reset_angle(0)

    PROPORTIONAL_GAIN = 1
    if distance < 0: # move backwards
        while robot.distance() > distance:
            reverseSpeed = -1 * robotSpeed        
            angle_correction = -1 * PROPORTIONAL_GAIN * gyro_sensor.angle()
            robot.drive(reverseSpeed, angle_correction) 
            wait(10)
    elif distance > 0: # move forwards             
        while robot.distance() < distance:
            angle_correction = -1 * PROPORTIONAL_GAIN * gyro_sensor.angle()
            robot.drive(robotSpeed, angle_correction) 
            wait(10)            
    robot.stop()

# gyrostrt()