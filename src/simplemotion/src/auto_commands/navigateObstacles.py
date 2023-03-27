# Author: Arjun Viswanathan
# Date created: 3/9/23
# Navigate around obstacles in front of stretch using LaserScan in autonomous control
# SimpleAvoid: performs avoidance while continuously going forward
# BetterAvoid: performs avoidance and follows you when you are above some avoid threshold

# TODO: write takeBetterAction function in BetterAvoid class

import rospy
from sensor_msgs.msg import LaserScan
import math
import time
import stretch_body.robot as sb

class SimpleAvoid:
    def __init__(self):
        self.robot = sb.Robot()
        self.robot.startup()

        self.base = self.robot.base
        self.start_time = time.time()

        self.v = 10.0
        self.a = 5.0
        self.timeout = 1

        self.distance = 1

        rospy.init_node('laser_scan')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.computeRegions)
        rospy.spin()

    def computeRegions(self, msg):
        # min_angle = -pi, max_angle = pi, and they both point in front of stretch, where x axis is
        # calculate a difference from min_angle using unit circle, and then use that to get range index
        fleft = int((math.pi/3) / msg.angle_increment)
        left = int((2*math.pi/3) / msg.angle_increment)
        fright = int((5*math.pi/3) / msg.angle_increment)
        right = int((4*math.pi/3) / msg.angle_increment)

        regions = {
        'fleft': min(min(msg.ranges[fleft:left]), 10) < self.distance,
        'front':  min(min(msg.ranges[0:fleft] + msg.ranges[fright:]), 10) < self.distance,
        'fright':  min(min(msg.ranges[right:fright]), 10) < self.distance
        }

        self.takeAction(regions)

    def takeAction(self, regions):
        xm = 0
        xr = 0

        if regions['front']:
            state_description = 'case 2 - front'
            xm = 0

            if regions['fright']:
                state_description = 'case 3 - front and fright'
                xr = 0.3
                xm = 0
            elif regions['fleft']:
                state_description = 'case 4 - front and fleft'
                xr = -0.3
                xm = 0
            elif regions['fleft'] and regions['fright']:
                state_description = 'case 5 - front and fleft and fright'
                xm = -0.1
                xr = 0
        elif regions['fleft'] or regions['fright']:
            state_description = 'case 8 - fleft or fright'
            xm = 0.1
        else:
            state_description = 'case 1 - nothing'
            xm = 0.1

        if xm != 0:
            self.move_base(xm)
        
        if xr != 0:
            self.rotate_base(xr)
            
        self.robot.push_command()
        time.sleep(0.1)

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
        # min_angle = -pi, max_angle = pi, and they both point in front of stretch, where x axis is
        # calculate a difference from min_angle using unit circle, and then use that to get range index
        fleft = int((math.pi/3) / msg.angle_increment)
        left = int((2*math.pi/3) / msg.angle_increment)
        fright = int((5*math.pi/3) / msg.angle_increment)
        right = int((4*math.pi/3) / msg.angle_increment)
        back = int((math.pi) / msg.angle_increment)

        regionsClose = {
        'left': min(min(msg.ranges[left:back]), 10) < self.distance,
        'fleft': min(min(msg.ranges[fleft:left]), 10) < self.distance,
        'front':  min(min(msg.ranges[0:fleft] + msg.ranges[fright:]), 10) < self.distance,
        'fright':  min(min(msg.ranges[right:fright]), 10) < self.distance,
        'right': min(min(msg.ranges[back:right]), 10) < self.distance
        }

        regionsFar = {
        'left': min(min(msg.ranges[left:back]), 10) < 2*self.distance,
        'fleft': min(min(msg.ranges[fleft:left]), 10) < 2*self.distance,
        'front':  min(min(msg.ranges[0:fleft] + msg.ranges[fright:]), 10) < 2*self.distance,
        'fright':  min(min(msg.ranges[right:fright]), 10) < 2*self.distance,
        'right': min(min(msg.ranges[back:right]), 10) < 2*self.distance
        }

        self.takeAction(regionsClose, regionsFar)

    def takeBetterAction(self, moveBack, moveForward):
        xm = 0
        xr = 0

        if moveBack['front']:
            state_description = 'case 2 - front'
            xm = 0

            if moveBack['fright']:
                state_description = 'case 3 - front and fright'
                xr = 0.3
                xm = 0
            elif moveBack['fleft']:
                state_description = 'case 4 - front and fleft'
                xr = -0.3
                xm = 0
            elif moveBack['fleft'] and moveBack['fright']:
                state_description = 'case 5 - front and fleft and fright'
                xm = -0.1
                xr = 0
        elif moveBack['fleft'] or moveBack['fright']:
            state_description = 'case 8 - fleft or fright'
            xm = 0.1
        else:
            state_description = 'case 1 - nothing'
            xm = 0.1

        if xm != 0:
            self.move_base(xm)
        
        if xr != 0:
            self.rotate_base(xr)
            
        self.robot.push_command()
        time.sleep(0.1)

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