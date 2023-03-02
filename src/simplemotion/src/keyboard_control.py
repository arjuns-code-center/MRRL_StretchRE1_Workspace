#!/usr/bin/env python
# Arjun Viswanathan
# 2/16/23
# Main script to move stretch with keyboard input

import sys, tty, termios
import time
import stretch_body.robot as sb
from stretch_body.hello_utils import *

print("========STRETCH Keyboard Controls========")
print("Use WASD to move the base")
print("Use Q and E to extend or retract the arm")
print("Use Z and C to move lift up and down")
print("Use N and M to pan head")
print("Use I and O to tilt head")
print("Use K and L to turn the wrist")
print("Use H and J to move gripper")
print("Use F and G to control wrist pitch")
print("Use V and B to control wrist roll")
print("Use T to stop robot")

class Stretch_Move:
    def __init__(self):
        # Define robot object and start it up
        self.robot = sb.Robot()
        self.robot.startup()

        # Initialize each part of robot
        self.arm = self.robot.arm
        self.base = self.robot.base
        self.lift = self.robot.lift
        self.endofarm = self.robot.end_of_arm # stores wrist_yaw, stretch_gripper, wrist_pitch, and wrist_roll
        self.head = self.robot.head

        # Initialize the params for each part
        self.base_v = self.base.params['motion']['max']['vel_m']
        self.base_a = self.base.params['motion']['max']['accel_m']

        self.v = 10.0
        self.a = 5.0
        
        self.gripper_v = 20.0
        self.gripper_a = 10.0

    def execCommand(self):
        self.robot.push_command()
        self.delay(0.05)

    def robot_stop(self):
        self.robot.stop()

    def delay(self, value):
        time.sleep(value)

    def move_base(self, x, wait=False):
        # Use distance
        self.base.translate_by(x_m=x, v_m=self.base_v, a_m=self.base_a)

        if wait:
            self.base.wait_until_at_setpoint()

    def rotate_base(self, theta, wait=False):
        # Use distance
        self.base.rotate_by(x_r=theta, v_r=self.v, a_r=self.a)

        if wait:
            self.base.wait_until_at_setpoint()

    def move_arm_absolute(self, position, interrupt):
        # Move to position at velocity and acceleration v, a
        self.arm.move_to(position, self.v, self.a)
        
        # Only wait until completion if interrupt signal not given
        if not interrupt:
            self.arm.wait_until_at_setpoint()

    def move_lift_absolute(self, position, interrupt):
        # Move to position at velocity and acceleration v, a
        self.lift.move_to(position, self.v, self.a)
        
        # Only wait until completion if interrupt signal not given
        if not interrupt:
            self.lift.wait_until_at_setpoint()

    def move_arm_incremental(self, distance):
        # Move by distance at velocity and acceleration v, a
        self.arm.move_by(distance, self.v, self.a)

    def move_lift_incremental(self, distance):
        self.lift.move_by(distance, self.v, self.a)

    def move_gripper(self, degrees):
        self.endofarm.move_by('stretch_gripper', deg_to_rad(degrees), self.gripper_v, self.gripper_a)

    def controlEOA(self, name, degrees):
        # possible params for name: wrist_yaw, wrist_pitch, wrist_roll
        self.endofarm.move_by(name, deg_to_rad(degrees), self.v, self.a)
        #self.wrist.go_to_pos(currentPos + distance)

    def controlHead(self, name, degrees):
        # possible params for name: head_pan, head_tilt
        self.head.move_by(name, deg_to_rad(degrees), self.v, self.a)

    def auto_box(self, move = 0.5, rotate = 1.7):
        print("Moving robot in a box")

        for i in range(4):
            self.move_base(x_m=move, wait=True)
            self.execCommand()
            
            self.rotate_base(x_r=rotate, wait=True)
            self.execCommand()

        print("Finished box!")

    def dynamixel_servo(self, name='/dev/hello-dynamixel-wrist'):
        servo = sb.dynamixel_XL430.DynamixelXL430(13, name, baud=115200)
        servo.startup()

        x = servo.get_pos()
        servo.go_to_pos(x-500)
        self.delay(1.0)

        x = servo.get_pos()
        servo.go_to_pos(x-500)
        self.delay(1.0)

        servo.stop()

class Keys:
    def __init__(self):
        self.quit = 0
        self.sm = Stretch_Move()

    def getch(self):
        # Keyboard file from stretch_core
        stdin_fd = 0
        # "Return a list containing the tty attributes for file descriptor
        # fd, as follows: [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]"
        # from https://docs.python.org/2/library/termios.html
        original_tty_attributes = termios.tcgetattr(stdin_fd)
        new_tty_attributes = termios.tcgetattr(stdin_fd)
        # Change the lflag (local modes) to turn off canonical mode
        new_tty_attributes[3] &= ~termios.ICANON
        # Set VMIN = 0 and VTIME > 0 for a timed read, as explained in:
        # http://unixwiz.net/techtips/termios-vmin-vtime.html
        new_tty_attributes[6][termios.VMIN] = b'\x00'
        new_tty_attributes[6][termios.VTIME] = b'\x01'

        new_tty_attributes[3] &= ~termios.ECHO
        try:
            termios.tcsetattr(stdin_fd, termios.TCSAFLUSH, new_tty_attributes)
            ch1 = sys.stdin.read(1)
            if ch1 == '\x1b':
                # special key pressed
                ch2 = sys.stdin.read(1)
                ch3 = sys.stdin.read(1)
                ch = ch1 + ch2 + ch3
            else:
                # not a special key
                ch = ch1
        finally:
            termios.tcsetattr(stdin_fd, termios.TCSAFLUSH, original_tty_attributes)

        return ch
    
    def getkeystroke(self):
        fd=sys.stdin.fileno()
        old_settings=termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())
            ch=sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)

        return ch

    def queueActions(self):
        key = self.getkeystroke()

        # Moving forward/backward
        if key == 'w' or key == 'W':
            self.sm.move_base(0.1)
        elif key == 's' or key == 'S':
            self.sm.move_base(-0.1)

        # Turning left/right
        if key == 'a' or key == 'A':
            self.sm.rotate_base(-0.2)
        elif key == 'd' or key == 'D':
            self.sm.rotate_base(0.2)

        # Arm extend/retract
        if key == 'q' or key == 'Q':
            self.sm.move_arm_incremental(0.02)
        elif key == 'e' or key == 'E':
            self.sm.move_arm_incremental(-0.02)

        # Lift up/down
        if key == 'z' or key == 'Z':
            self.sm.move_lift_incremental(0.02)
        elif key == 'c' or key == 'C':
            self.sm.move_lift_incremental(-0.02)

        if key == 'k' or key == 'K':
            self.sm.controlEOA('wrist_yaw', 5)
        elif key == 'l' or key == 'L':
            self.sm.controlEOA('wrist_yaw', -5)

        if key == 'f' or key == 'F':
            self.sm.controlEOA('wrist_pitch', 5)
        elif key == 'g' or key == 'G':
            self.sm.controlEOA('wrist_pitch', -5)

        if key == 'v' or key == 'V':
            self.sm.controlEOA('wrist_roll', 5)
        elif key == 'b' or key == 'B':
            self.sm.controlEOA('wrist_roll', -5)

        if key == 'n' or key == 'N':
            self.sm.controlHead('head_pan', 10)
        elif key == 'm' or key == 'M':
            self.sm.controlHead('head_pan', -10)

        if key == 'i' or key == 'I':
            self.sm.controlHead('head_tilt', 30)
        elif key == 'o' or key == 'O':
            self.sm.controlHead('head_tilt', -30)

        if key == 'h' or key == 'H':
            self.sm.move_gripper(90)
        elif key == 'j' or key == 'J':
            self.sm.move_gripper(-90)

        if key == 't' or key == 'T':
            print("Stopping robot")
            self.sm.robot_stop()
            self.quit = 1

    def execCommand(self):
        self.sm.execCommand()

    def isStopped(self):
        return self.quit

kb = Keys()

while not kb.isStopped():
    kb.queueActions()
    kb.execCommand()
    time.sleep(0.1)