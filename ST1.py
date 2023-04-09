def start():
    for i in range(10):
        ev3.speaker.beep()
start()

def inch_to_mm(inch):
    mm=inch*25.4
    return mm

def box_dist(box_num):
    box_d_inches=(6*(box_num-6))-3
    return box_d_inches

def turn(a):
    angle = a # degrees
    s = 200 # mm/s
    
    #callibrates sensor
    robot.reset() 
    gyro_sensor.reset_angle(0)

    while gyro_sensor.angle() <= angle:
        left_motor.run(speed=s)
        right_motor.run(speed=(-1 * s))
        
    left_motor.brake()
    right_motor.brake()
    ev3.speaker.beep()     

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

def obsavoider(d):
    distn=d
    robot.reset()
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
            
            wait(5000)
        # Checks if the robot has reached the required distance
        if robot.distance()>=distn:
            break
        print(robot.distance())

def main_subtask_1():
    box=7 #put the number of the box here
    x=inch_to_mm(box_d_inches(box))
    y=inch_to_mm(30+2)

    x_return=inch_to_mm(96-box_d_inches(box)+3)

    obsavoider(y)
    turn(90)
    obsavoider(x)
    wait(5000)
    obsavoider(x_return)
    turn(90)
    obsavoider(y)

