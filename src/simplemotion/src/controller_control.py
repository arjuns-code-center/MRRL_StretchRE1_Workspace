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

def main():
    xbox_controller = xc.XboxController()
    xbox_controller.start()
    sm = Stretch_Move()

    while True:
        controller_state = xbox_controller.get_state()

        # TODO: finish all action listners
        if not robot.is_calibrated():
            manage_calibration(robot, controller_state)
        else:
            manage_base(robot, controller_state)
            manage_lift_arm(robot, controller_state)
            manage_end_of_arm(robot, controller_state)
            manage_head(robot, controller_state)
            manage_stow(robot, controller_state)
            manage_image_capture(robot, controller_state)
        manage_shutdown(robot, controller_state)
        robot.push_command()
        time.sleep(0.05)