#!/usr/bin/env python
# Arjun Viswanathan
# 2/26/23
# Main script to move stretch with keyboard input but this time uses pygame module to move multiple components at the same time

import time
import stretch_body.robot as sb
from stretch_body.hello_utils import *
import stretch_body.xbox_controller as xc


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

    def move_base(self, x):
        # Use distance
        self.base.translate_by(x_m=x, v_m=self.base_v, a_m=self.base_a)

    def rotate_base(self, theta):
        # Use distance
        self.base.rotate_by(x_r=theta, v_r=self.v, a_r=self.a)

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
        #base_v = robot.base.params['motion']['max']['vel_m']
        #base_a = robot.base.params['motion']['max']['accel_m']
        #base_w = robot.base.params['rotation']['max']['vel_m']
        #base_r = robot.base.params['rotation']['max']['accel_m']

        #move_delay = move / base_v
        #rot_delay = move / base_w

        print("Moving robot in a box")

        for i in range(4):
            self.robot.base.translate_by(x_m=move)
            self.robot.push_command()
            self.robot.base.wait_until_at_setpoint()
            #time.sleep(move_delay)
            
            #for j in range(8):
            self.robot.base.rotate_by(x_r=rotate)
            self.robot.push_command()
            #time.sleep(rot_delay)
            self.robot.base.wait_until_at_setpoint()

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

        self.key_state = {'w': False, 'a': False, 's': False, 'd': False,
                        'q': False, 'e': False, 
                        'z': False, 'c': False,
                        'n': False, 'm': False, 
                        'i': False, 'o': False, 
                        'k': False, 'l': False, 
                        'h': False, 'j': False,
                        'f': False, 'g': False, 
                        'v': False, 'b': False,
                        't': False}
        
    def setKeyStates(self):
        keys = pygame.key.get_pressed()

        self.key_state['w'] = keys[pygame.K_w]
        self.key_state['a'] = keys[pygame.K_a]
        self.key_state['s'] = keys[pygame.K_s]
        self.key_state['d'] = keys[pygame.K_d]

        self.key_state['q'] = keys[pygame.K_q]
        self.key_state['e'] = keys[pygame.K_e]
        self.key_state['z'] = keys[pygame.K_z]
        self.key_state['c'] = keys[pygame.K_c]

        self.key_state['n'] = keys[pygame.K_n]
        self.key_state['m'] = keys[pygame.K_m]
        self.key_state['i'] = keys[pygame.K_i]
        self.key_state['o'] = keys[pygame.K_o]

        self.key_state['k'] = keys[pygame.K_k]
        self.key_state['l'] = keys[pygame.K_l]
        self.key_state['h'] = keys[pygame.K_h]
        self.key_state['j'] = keys[pygame.K_j]

        self.key_state['f'] = keys[pygame.K_f]
        self.key_state['g'] = keys[pygame.K_g]
        self.key_state['v'] = keys[pygame.K_v]
        self.key_state['b'] = keys[pygame.K_b]

        self.key_state['t'] = keys[pygame.K_t]
    
    def queueActions(self):
        # Moving forward/backward
        if self.key_state['w']:
            self.sm.move_base(0.1)
        
        if self.key_state['s']:
            self.sm.move_base(-0.1)

        # Turning left/right
        if self.key_state['a']:
            self.sm.rotate_base(-0.2)
        
        if self.key_state['d']:
            self.sm.rotate_base(0.2)

        # Arm extend/retract
        if self.key_state['q']:
            self.sm.move_arm_incremental(0.02)
        
        if self.key_state['e']:
            self.sm.move_arm_incremental(-0.02)

        # Lift up/down
        if self.key_state['z']:
            self.sm.move_lift_incremental(0.02)
        
        if self.key_state['c']:
            self.sm.move_lift_incremental(-0.02)

        if self.key_state['k']:
            self.sm.controlEOA('wrist_yaw', 5)
        
        if self.key_state['l']:
            self.sm.controlEOA('wrist_yaw', -5)

        if self.key_state['f']:
            self.sm.controlEOA('wrist_pitch', 5)
        
        if self.key_state['g']:
            self.sm.controlEOA('wrist_pitch', -5)

        if self.key_state['v']:
            self.sm.controlEOA('wrist_roll', 5)
        
        if self.key_state['b']:
            self.sm.controlEOA('wrist_roll', -5)

        if self.key_state['n']:
            self.sm.controlHead('head_pan', 10)
        
        if self.key_state['m']:
            self.sm.controlHead('head_pan', -10)

        if self.key_state['i']:
            self.sm.controlHead('head_tilt', 30)
        
        if self.key_state['o']:
            self.sm.controlHead('head_tilt', -30)

        if self.key_state['h']:
            self.sm.move_gripper(90)
        
        if self.key_state['j']:
            self.sm.move_gripper(-90)

        if self.key_state['t']:
            print("Stopping robot")
            self.sm.robot_stop()
            self.quit = 1

    def execCommand(self):
        self.sm.execCommand()

    def isStopped(self):
        return self.quit

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

kb = Keys()

while not kb.isStopped():
    kb.setKeyStates()
    kb.queueActions()
    kb.execCommand()
    time.sleep(0.1)