# Author: Arjun Viswanathan
# Date created: 3/9/23
# Last modified date: 4/6/23
# Summary: Navigate around obstacles in front of stretch using LiDAR in autonomous mode
# SimpleAvoid: performs avoidance while continuously going forward
# BetterAvoid: performs avoidance and considers previous states to navigate better

# How to run the file:
# rosrun simplemotion navigateObstacles.py --type=<SPECIFY TYPE>

# TODO: test out BetterAvoid

# Import system packages
import math
import time
import argparse

# Import ROS specific packages
import rospy
from sensor_msgs.msg import LaserScan
import stretch_body.robot as sb

class SimpleAvoid:
    def __init__(self):
        self.robot = sb.Robot()
        self.robot.startup()

        self.base = self.robot.base

        self.v = 10.0 #self.base.params['motion']['max']['vel_m']
        self.a = 5.0 #self.base.params['motion']['max']['accel_m']
        self.timeout = 1

        self.distance = 1.0
        self.ignore = 0.25

        rospy.init_node('laser_scan')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.computeRegions)
        rospy.spin()

    def computeRegions(self, msg):
        # min_angle = -pi, max_angle = pi, and they both point in front of stretch, where x axis is
        # calculate a difference from min_angle using unit circle, and then use that to get range index
        fleft = int((math.pi/4) / msg.angle_increment)
        left = int((math.pi/2) / msg.angle_increment)
        fright = int((7*math.pi/4) / msg.angle_increment)
        right = int((3*math.pi/2) / msg.angle_increment)

        fleftClosest = min(i for i in msg.ranges[fleft:left] if i > self.ignore)
        frontClosest = min(i for i in msg.ranges[0:fleft] + msg.ranges[fright:] if i > self.ignore)
        frightClosest = min(i for i in msg.ranges[right:fright] if i > self.ignore)

        regions = {
        'fleft': fleftClosest < self.distance,
        'front': frontClosest < self.distance,
        'fright': frightClosest < self.distance
        }

        self.takeAction(regions)

    def takeAction(self, regions):
        # Simple case where it just reflexively takes an action to avoid obstacles
        # Major flaw: can deadlock between 2 states 
        xm = 0
        xr = 0

        if regions['fright']:
            state_description = 'case 2 - fright'
            xr = 0.15
        elif regions['fleft']:
            state_description = 'case 3 - fleft'
            xr = -0.15
        elif regions['front']:
            state_description = 'case 4 - front (turning right)'
            xr = -0.15
        else:
            state_description = 'case 1 - nothing in front'
            xm = 0.15

        print(state_description)

        if xm != 0:
            self.move_base(xm)
            self.robot.push_command()
        
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
    def __init__(self):
        self.robot = sb.Robot()
        self.robot.startup()

        self.base = self.robot.base

        self.v = 10.0
        self.a = 5.0
        self.timeout = 1

        self.distance = 0.75
        self.ignore = 0.25

        self.currentState = None
        self.previousState = None
        self.currentStateChanged = True

        rospy.init_node('laser_scan')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.computeBetterRegions)
        rospy.spin()

    def computeBetterRegions(self, msg):
        # min_angle = -pi, max_angle = pi, and they both point in front of stretch, where x axis is
        # calculate a difference from min_angle using unit circle, and then use that to get range index
        fleft = int((math.pi/4) / msg.angle_increment)
        left = int((math.pi/2) / msg.angle_increment)
        fright = int((7*math.pi/4) / msg.angle_increment)
        right = int((3*math.pi/2) / msg.angle_increment)

        fleftClosest = min(i for i in msg.ranges[fleft:left] if i > self.ignore)
        frontClosest = min(i for i in msg.ranges[0:fleft] + msg.ranges[fright:] if i > self.ignore)
        frightClosest = min(i for i in msg.ranges[right:fright] if i > self.ignore)
        backClosest = min(i for i in msg.ranges[left:right] if i > self.ignore)

        # Here we want to backup and move away from obstacles when they get too close
        regions = {
        'fleft_backup': fleftClosest < self.distance,
        'front_backup':  frontClosest < self.distance,
        'fright_backup':  frightClosest < self.distance,
        'back_gofront': backClosest < self.distance,

        'fleft': fleftClosest < 2*self.distance,
        'front':  frontClosest < 2*self.distance,
        'fright':  frightClosest < 2*self.distance,
        'back': backClosest < 2*self.distance
        }

        self.takeBetterAction(regions)
        if self.currentStateChanged: # only print the state when it changes
            print(self.currentState)
            self.previousState = self.currentState

    def takeBetterAction(self, regions):
        # Keep track of a previous state so we do not deadlock between 2 actions forever
        # Stretch can detect an obstacle in fright, turn left to avoid, and detect in front, turn right to avoid, and continue doing this forever
        # Based on the previous state, actions will change
        xm = 0
        xr = 0
        tempState = self.currentState

        if regions['fright']:
            tempState = 'fright'

            if self.previousState == 'front':
                xr = -0.15
            else:
                xr = 0.15
        elif regions['fleft']:
            tempState = 'fleft'

            if self.previousState == 'front':
                xr = 0.15
            else:
                xr = -0.15
        elif regions['front']:
            tempState = 'front'

            if self.previousState == 'fright':
                xr = -0.15
            elif self.previousState == 'fleft':
                xr = 0.15
            else:
                xr = -0.15
        elif regions['fright_backup'] or regions['fleft_backup'] or regions['front_backup']:
            tempState = 'backup'

            if self.previousState == 'gofront':
                xr = -0.15
            else:
                xm = -0.15
        elif regions['back_gofront']:
            tempState = 'gofront'

            if self.previousState == 'fright_backup' or self.previousState == 'fleft_backup' or self.previousState == 'front_backup':
                xr = -0.15
            else:
                xm = 0.15
        else:
            tempState = 'nothing'
            xm = 0.15

        # If state did not change, we do not update previous state. Or else both can become the same and we lose our previous information
        if tempState == self.currentState:
            self.currentStateChanged = False
        else:
            self.currentStateChanged = True

        if xm != 0:
            self.move_base(xm)
            self.robot.push_command()
        
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
    start_time = time.ctime()
    print("{}: Starting Navigation Algorithm...".format(start_time))

    args = argparse.ArgumentParser()
    args.add_argument('--algotype', default='simple', type=str, help='what avoidance algorithm to run')
    args = args.parse_args()
    algorithmType = args.algotype

    if algorithmType == 'simple':
        print("Using SimpleAvoid algorithm to navigate obstacles")
        SimpleAvoid()
    elif algorithmType == 'better':
        print("Using BetterAvoid algorithm to navigate obstacles")
        BetterAvoid()