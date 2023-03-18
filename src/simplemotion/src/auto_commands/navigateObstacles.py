# Author: Arjun Viswanathan
# Date created: 3/9/23
# Navigate around obstacles in front of stretch using LaserScan in autonomous control
# SimpleAvoid: performs avoidance while continuously going forward
# BetterAvoid: performs avoidance and follows you when you are above some avoid threshold

# TODO: write takeBetterAction function in BetterAvoid class

import rospy
from sensor_msgs.msg import LaserScan
import time

class SimpleAvoid:
    def __init__(self, robot):
        self.start_time = time.time()

        self.robot = robot
        self.base = robot.base

        self.v = 10.0
        self.a = 5.0
        self.timeout = 1

        self.distance = 1

        print("Navigating robot around obstacles")

        rospy.init_node('laser_navigation')
        sub = rospy.Subsciber('/m2wr/laser/scan', LaserScan, self.computeRegions)
        rospy.spin()

    def computeRegions(self, msg):
        regions = {
        'fright': min(min(msg.ranges[144:287]), 10) < self.distance,
        'front':  min(min(msg.ranges[288:431]), 10) < self.distance,
        'fleft':  min(min(msg.ranges[432:575]), 10) < self.distance
        }

        self.takeAction(regions)

    def takeAction(self, regions):
        xm = 0.5
        xr = 0

        if regions['front']:
            state_description = 'case 2 - front'

            if regions['fright']:
                state_description = 'case 3 - front and fright'
                xr = -0.3
            elif regions['fleft']:
                state_description = 'case 4 - front and fleft'
                xr = 0.3
            elif regions['fleft'] and regions['fright']:
                state_description = 'case 5 - front and fleft and fright'
                xm = -0.5
        elif regions['fleft'] or regions['fright']:
            state_description = 'case 8 - fleft or fright'
        else:
            state_description = 'case 1 - nothing'

        # if regions['front'] > self.distance and regions['fleft'] > self.distance and regions['fright'] > self.distance:
        #     state_description = 'case 1 - nothing'
        # elif regions['front'] < self.distance and regions['fleft'] > self.distance and regions['fright'] > self.distance:
        #     state_description = 'case 2 - front'
        #     xr = -0.3
        # elif regions['front'] > self.distance and regions['fleft'] > self.distance and regions['fright'] < self.distance:
        #     state_description = 'case 3 - fright'
        #     xr = -0.3
        # elif regions['front'] > self.distance and regions['fleft'] < self.distance and regions['fright'] > self.distance:
        #     state_description = 'case 4 - fleft'
        #     xr = 0.3
        # elif regions['front'] < self.distance and regions['fleft'] > self.distance and regions['fright'] < self.distance:
        #     state_description = 'case 5 - front and fright'
        #     xr = -0.3
        # elif regions['front'] < self.distance and regions['fleft'] < self.distance and regions['fright'] > self.distance:
        #     state_description = 'case 6 - front and fleft'
        #     xr = 0.3
        # elif regions['front'] < self.distance and regions['fleft'] < self.distance and regions['fright'] < self.distance:
        #     state_description = 'case 7 - front and fleft and fright'
        #     xm = -0.5
        # elif regions['front'] > self.distance and regions['fleft'] < self.distance and regions['fright'] < self.distance:
        #     state_description = 'case 8 - fleft and fright'
        # else:
        #     state_description = 'unknown case'

        print(state_description)
        self.move_base(xm)
        self.rotate_base(xr)
        self.robot.push_command()

    def move_base(self, x, wait=False):
        # Use distance
        self.base.translate_by(x_m=x, v_m=self.v, a_m=self.a)

        if wait:
            self.base.wait_until_at_setpoint(self.timeout)

    def rotate_base(self, theta, wait=False):
        # Use distance
        self.base.rotate_by(x_r=theta, v_r=self.v, a_r=self.a)

        if wait:
            self.base.wait_until_at_setpoint(self.timeout)

class BetterAvoid:
    def __init__(self, robot):
        self.start_time = time.time()

        self.robot = robot
        self.base = robot.base

        self.v = 10.0
        self.a = 5.0
        self.timeout = 1

        self.distance = 1

        print("Navigating robot around obstacles more robustly")

        rospy.init_node('better_laser_navigation')
        sub = rospy.Subsciber('/m2wr/laser/scan', LaserScan, self.computeBetterRegions)
        rospy.spin()

    def computeBetterRegions(self, msg):
        regions = {
        'right': min(min(msg.ranges[0:143]), 10) < self.distance,
        'fright': min(min(msg.ranges[144:287]), 10) < self.distance,
        'front':  min(min(msg.ranges[288:431]), 10) < self.distance,
        'fleft':  min(min(msg.ranges[432:575]), 10) < self.distance,
        'left': min(min(msg.ranges[576:719]), 10) < self.distance
        }

        self.takeBetterAction(regions)

    def takeBetterAction(self, regions):
        xm = 0.5
        xr = 0

        if regions['front']:
            state_description = 'case 2 - front'

            if regions['fright']:
                state_description = 'case 3 - front and fright'
                xr = -0.3
            elif regions['fleft']:
                state_description = 'case 4 - front and fleft'
                xr = 0.3
            elif regions['fleft'] and regions['fright']:
                state_description = 'case 5 - front and fleft and fright'
                xm = -0.5
        elif regions['fleft'] or regions['fright']:
            state_description = 'case 8 - fleft or fright'
        else:
            state_description = 'case 1 - nothing'

        print(state_description)
        self.move_base(xm)
        self.rotate_base(xr)
        self.robot.push_command()

    def move_base(self, x, wait=False):
        # Use distance
        self.base.translate_by(x_m=x, v_m=self.v, a_m=self.a)

        if wait:
            self.base.wait_until_at_setpoint(self.timeout)

    def rotate_base(self, theta, wait=False):
        # Use distance
        self.base.rotate_by(x_r=theta, v_r=self.v, a_r=self.a)

        if wait:
            self.base.wait_until_at_setpoint(self.timeout)

if __name__ == "__main__":
    SimpleAvoid()
    #BetterAvoid()