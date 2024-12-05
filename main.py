!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pixycamev3.pixy2 import Pixy2

Initialize devices
ev3 = EV3Brick()
left_motor = Motor(Port.A)   # Left motor for wheels
right_motor = Motor(Port.D)  # Right motor for wheels
claw_motor = Motor(Port.C)   # Motor for claw
pixy = Pixy2(port=1, i2c_address=0x54)   # Pixy2 camera in sensor port 1
ev3.speaker.beep()

# Constants
SPEED = 250  # Speed in degrees per second
CLAW_SPEED = 300  # Speed for the claw
PIXEL_CENTER = 160  # Center pixel in Pixy2 x-axis
CENTER_THRESHOLD = 20  # How close to center is acceptable
APPROACH_DISTANCE = 400  # How close the cup should be in x-pixels

def get_blocks():
    """Fetch block data from Pixy2 camera."""
    #blocks = BlockArray(100)
    frame  = 0
    #count = pixy.get_blocks(100, blocks)
    nr_blocks, blocks = pixy.get_blocks(1, 1)
    # if blocks:
    #     for block in blocks:
    #         return block.signature, block.x, block.y
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
            if signature in (1, 2):  # Signature 1: Red, Signature 2: Blue
                left_motor.stop()
                right_motor.stop()
                ev3.speaker.beep()  # Beep to confirm detection
                return signature  # Return the detected signature
        wait(100)

def approach_cup():
    claw_motor.run_time(CLAW_SPEED, 4000, Stop.HOLD)  # Open the Claw
    """Approach the detected cup."""
    while True:
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
    claw_motor.run_time(-CLAW_SPEED, 4000, Stop.HOLD)  # Close the claw
    left_motor.run_time(-SPEED, 1000, Stop.HOLD)  # Reverse
    right_motor.run_time(-SPEED, 1000, Stop.HOLD)

claw_motor.run_time(-CLAW_SPEED, 4000, Stop.HOLD)  # Close the claw
while True:
    signature = scan_for_cup()
    if signature:  # If a cup is detected
        approach_cup()
        pick_up_cup()
        break  # Stop after picking up the first cup
