#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pixycamev3.pixy2 import (Pixy2, Pixy2ConnectionError, Pixy2DataError)
from time import sleep

# Initialize devices
ev3 = EV3Brick()
left_motor = Motor(Port.A)   # Left motor for wheels
right_motor = Motor(Port.D)  # Right motor for wheels
claw_motor = Motor(Port.C)   # Motor for claw
pixy = Pixy2(port=1, i2c_address=0x54)   # Pixy2 camera in sensor port 1
light = ColorSensor
gyro_sensor = GyroSensor(Port.S2)

#Constants
SPEED = 250  # Speed in degrees per second
CLAW_SPEED = 300  # Speed for the claw
PIXEL_CENTER = 160  # Center pixel in Pixy2 x-axis
CENTER_THRESHOLD = 20  # How close to center is acceptable
APPROACH_DISTANCE = 130  # Width of cup in pixels
current_angle = gyro_sensor.angle()

def get_blocks():
    """Fetch block data from Pixy2 camera."""
    
    nr_blocks, blocks = pixy.get_blocks(3, 10)
    if nr_blocks >= 1:
        return blocks[0]    
    else:
        return

def scan_for_cup():
    """Scan for a red or blue cup by rotating the robot."""
    scan_direction = 1  # Start scanning clockwise
    while True:
        # Rotate the robot
        left_motor.run(scan_direction * 200)
        right_motor.run(-scan_direction * 200)
        # Reverse scanning direction periodically
        scan_direction = -scan_direction  # Toggle direction
        wait(100)
        
        # Check for blocks using Pixy2
        block = get_blocks()
        if block:
            ev3.speaker.beep()  # Beep when a block is detected
            signature = block.sig
            if signature in (1,2):  # Signature 1: Red, Signature 2: Blue
                left_motor.stop()
                right_motor.stop()
                ev3.speaker.beep()  # Beep to confirm detection
                return signature  # Return the detected signature
        wait(100)

def approach_cup():
    """Approach the detected cup."""
    while True:
        current_angle = gyro_sensor.angle()
        block = get_blocks()
        if block:
            x = block.x_center
            y = block.y_center
            z = block.width
            # Center the robot with the cup
            if x < PIXEL_CENTER - CENTER_THRESHOLD:
                left_motor.run(-SPEED // 2)
                right_motor.run(SPEED // 2)  # Turn left
            elif x > PIXEL_CENTER + CENTER_THRESHOLD:
                left_motor.run(SPEED // 2)
                right_motor.run(-SPEED // 2)  # Turn right
            else:
                left_motor.run(SPEED)
                right_motor.run(SPEED)  # Drive straight
            # Stop if close enough
            if z >= APPROACH_DISTANCE:
                left_motor.stop()
                right_motor.stop()
                return
        else:
            # If the block is lost, rescan
            scan_for_cup()
        wait(100)

def pick_up_cup():

    """Pick up the cup with the claw."""
    current_angle = gyro_sensor.angle()
    claw_motor.run_time(-CLAW_SPEED, 4000, Stop.HOLD)  # Close the claw

def take_cup_to_blue_base():
   while True:
    current_angle = gyro_sensor.angle()
    degrees_to_spin = -current_angle  
    left_motor.run_angle(SPEED, degrees_to_spin, Stop.HOLD, wait=True)
    right_motor.run_angle(-SPEED, degrees_to_spin, Stop.HOLD, wait=True)
    gyro_sensor.reset_angle(0)
    if current_angle == 0:
        left_motor.run(-SPEED)
        right_motor.run(-SPEED)
        wait(5000)
        left_motor.stop() 
        right_motor.stop()
        break #Not sure if this goes in line with the if statement or right here

def drop_cup_off_at_blue_base():
    while True:
        current_angle = gyro_sensor.angle()
        target angle = 180
        degrees_to_spin = target_angle - current_angle
        left_motor.run_angle(SPEED, degrees_to_spin, Stop.HOLD, wait=True)
        right_motor.run_angle(-SPEED, degrees_to_spin, Stop.HOLD, wait=True)
         if current_angle == 180:
            left_motor.run(-SPEED)
            right_motor.run(-SPEED)
            wait(1000)
            left_motor.stop() 
            right_motor.stop()
            break #Not sure if this goes in line with the if statement or right here

def spin_if_blue():
    while True:
        current_angle = gyro_sensor.angle()
        degrees_to_spin = - current_angle

        left_motor.run_angle(SPEED, degrees_to_spin, Stop.HOLD, wait=True)
        right_motor.run_angle(-SPEED, degrees_to_spin, Stop.HOLD, wait=True)
        gyro_sensor.reset_angle(0)
        if current_angle == 0:
            break
        

def take_cup_to_red_base():
   while True:
    current_angle = gyro_sensor.angle()
    degrees_to_spin = -current_angle  
    left_motor.run_angle(SPEED, degrees_to_spin, Stop.HOLD, wait=True)
    right_motor.run_angle(-SPEED, degrees_to_spin, Stop.HOLD, wait=True)
    gyro_sensor.reset_angle(0)
    if current_angle == 0:
        left_motor.run(SPEED)
        right_motor.run(SPEED)
        wait(5000)
        left_motor.stop() 
        right_motor.stop()
    break

def drop_cup_off_at_red_base():
    claw_motor.run_time(CLAW_SPEED, 4000, Stop.HOLD)  # Open the Claw
    wait (1000)
    left_motor.run(-SPEED)
    right_motor.run(-SPEED)
    wait (7000) # Adjust this 
        
    
def open_the_claw():
    claw_motor.run_time(CLAW_SPEED, 4000, Stop.HOLD)  # Open the Claw

def close_claws():
    claw_motor.run_time(-CLAW_SPEED, 4000, Stop.HOLD)  # Close the claw

while True:
    current_angle = gyro_sensor.angle
    print(current_angle)
    wait(10000)
    signature = scan_for_cup()
    if signature: 
        claw_motor.run_time(CLAW_SPEED,4000,Stop.HOLD) #Open the claw (maybe replace this with a function)
        if signature == 1:
            approach_cup()
            pick_up_cup()
            # take_cup_to_red_base()
            # go_forward()
            # drop_cup_off()
        elif signature == 2:
            approach_cup()
            pick_up_cup()
            take_cup_to_blue_base()
            drop_cup_off_at_blue_base()
            spin_if_blue()
        
