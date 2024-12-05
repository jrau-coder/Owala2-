#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import Motor
# from pybricks.parameters import Port, Stop
# from pybricks.tools import wait
# from pybricks.robotics import DriveBase
# from pixycamev3.pixy2 import Pixy2

# Initialize devices
# ev3 = EV3Brick()
# left_motor = Motor(Port.A)   # Left motor for wheels
# right_motor = Motor(Port.D)  # Right motor for wheels
# claw_motor = Motor(Port.C)   # Motor for claw
# pixy = Pixy2(port=1, i2c_address=0x54)   # Pixy2 camera in sensor port 1
# ev3.speaker.beep()

from time import sleep
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port

from pixy2_pybricks import (Pixy2,
                            Pixy2ConnectionError,
                            Pixy2DataError)


def limit_speed(speed):
    """Limit speed in range [-900,900]."""
    if speed > 900:
        speed = 900
    elif speed < -900:
        speed = -900
    return speed

def main():
    # Objects for ev3-brick, motors and Pixy2 camera
    ev3 = EV3Brick()
    rmotor = Motor(Port.D)
    lmotor = Motor(Port.A)
    pixy2 = Pixy2(port=1, i2c_address=0x54)
    
    # Signature we're interesed in (SIG1)
    sig = 1
    
    # Defining constants
    X_REF = 158  # X-coordinate of referencepoint
    Y_REF = 150  # Y-coordinate of referencepoint
    KP = 0.3     # Proportional constant PID-controller
    KI = 0.01    # Integral constant PID-controller
    KD = 0.005   # Derivative constant PID-controller
    GAIN = 10    # Gain for motorspeed
    
    # Initializing PID variables
    integral_x = 0
    derivative_x = 0
    last_dx = 0
    integral_y = 0
    derivative_y = 0
    last_dy = 0
    
    while not ev3.buttons.pressed():
        # Read data from Pixy2 (only largest object)
        try:
            nr_blocks, blocks = pixy2.get_blocks(sig, 1)
            # Parse data
            if nr_blocks > 0:
                if sig == blocks[0].sig:
                    # SIG1 detected, control motors
                    x = blocks[0].x_center         # X-centroid of largest SIG1-object
                    y = blocks[0].y_center         # Y-centroid of largest SIG1-object
                    dx = X_REF - x                 # Error in reference to X_REF
                    integral_x = integral_x + dx   # Calculate integral for PID
                    derivative_x = dx - last_dx    # Calculate derivative for PID
                    speed_x = KP*dx + KI*integral_x + KD*derivative_x  # Speed X-direction
                    dy = Y_REF - y                 # Error in reference to Y_REF
                    integral_y = integral_y + dy   # Calculate integral for PID
                    derivative_y = dy - last_dy    # Calculate derivative for PID
                    speed_y = KP*dy + KI*integral_y + KD*derivative_y  # Speed Y-direction
                    # Calculate motorspeed out of speed_x and speed_y
                    # Use GAIN otherwise speed will be to slow,
                    # but limit in range [-1000,1000]
                    rspeed = limit_speed(GAIN*(speed_y - speed_x))
                    lspeed = limit_speed(GAIN*(speed_y + speed_x))
                    rmotor.run(round(rspeed))
                    lmotor.run(round(lspeed))
                    last_dx = dx                  # Set last error for x
                    last_dy = dy                  # Set last error for y
                else:
                    # SIG1 not detected, stop motors
                    rmotor.stop()
                    lmotor.stop()
                    last_dx = 0
                    last_dy = 0
        except Pixy2ConnectionError:
            # No data, stop program and check the connection of Pixy2
            print('Check connection Pixy2!')
            break
        except Pixy2DataError:
            # Data error, try reading again
            pass
    
    # Button pressed, stop motors, end of program
    rmotor.stop()
    lmotor.stop()


if __name__ == '__main__':
    main()


# # Constants
# SPEED = 250  # Speed in degrees per second
# CLAW_SPEED = 300  # Speed for the claw
# PIXEL_CENTER = 160  # Center pixel in Pixy2 x-axis
# CENTER_THRESHOLD = 20  # How close to center is acceptable
# APPROACH_DISTANCE = 400  # How close the cup should be in x-pixels


# def get_blocks():
#     """Fetch block data from Pixy2 camera."""
#     #blocks = BlockArray(100)
#     frame  = 0
#     #count = pixy.get_blocks(100, blocks)
#     nr_blocks, blocks = pixy.get_blocks(1, 1)
#     # if blocks:
#     #     for block in blocks:
#     #         return block.signature, block.x, block.y
#     if nr_blocks >= 1:
#         return blocks[0]    
#     else:
#         return

# def scan_for_cup():
#     """Scan for a red or blue cup by rotating the robot."""
#     scan_direction = 1  # Start scanning clockwise
#     while True:
#         # Rotate the robot
#         left_motor.run(scan_direction * 300)
#         right_motor.run(-scan_direction * 300)
#         # Reverse scanning direction periodically
#         scan_direction = -scan_direction  # Toggle direction
#         wait(100)
        
#         # Check for blocks using Pixy2
#         block = get_blocks()
#         if block:
#             ev3.speaker.beep()  # Beep when a block is detected
#             signature = block.sig
#             if signature in (1, 2):  # Signature 1: Red, Signature 2: Blue
#                 left_motor.stop()
#                 right_motor.stop()
#                 ev3.speaker.beep()  # Beep to confirm detection
#                 return signature  # Return the detected signature
#         wait(100)

# def approach_cup():
#     claw_motor.run_time(CLAW_SPEED, 4000, Stop.HOLD)  # Open the Claw
#     """Approach the detected cup."""
#     while True:
#         block = get_blocks()
#         if block:
#             x = block.x_center
#             y = block.y_center
#             # Center the robot with the cup
#             if x < PIXEL_CENTER - CENTER_THRESHOLD:
#                 left_motor.run(-SPEED // 2)
#                 right_motor.run(SPEED // 2)  # Turn left
#             elif x > PIXEL_CENTER + CENTER_THRESHOLD:
#                 left_motor.run(SPEED // 2)
#                 right_motor.run(-SPEED // 2)  # Turn right
#             else:
#                 left_motor.run(SPEED)
#                 right_motor.run(SPEED)  # Drive straight
#             # Stop if close enough
#             if y >= APPROACH_DISTANCE:
#                 left_motor.stop()
#                 right_motor.stop()
#                 return
#         else:
#             # If the block is lost, rescan
#             scan_for_cup()
#         wait(100)

# def pick_up_cup():

#     """Pick up the cup with the claw."""
#     claw_motor.run_time(-CLAW_SPEED, 4000, Stop.HOLD)  # Close the claw
#     left_motor.run_time(-SPEED, 1000, Stop.HOLD)  # Reverse
#     right_motor.run_time(-SPEED, 1000, Stop.HOLD)

# claw_motor.run_time(-CLAW_SPEED, 4000, Stop.HOLD)  # Close the claw
# while True:
#     signature = scan_for_cup()
#     if signature:  # If a cup is detected
#         approach_cup()
#         pick_up_cup()
#         break  # Stop after picking up the first cup