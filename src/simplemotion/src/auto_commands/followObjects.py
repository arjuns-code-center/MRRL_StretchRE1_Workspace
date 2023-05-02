# Author: Arjun Viswanathan
# Date created: 4/13/23
# Last modified date: 5/1/23
# Summary: follows a single object in front of it using only the LiDAR and intelligent decision making

# How to run from command line:
# rosrun simplemotion followObjects.py --timer=0
# For integration with keyboard_teleop, nothing to be done

# TODO: Incorporate computer vision 

# Import system packages
import math
import time
import argparse

# Import ROS specific packages
import rospy
from sensor_msgs.msg import LaserScan
import stretch_body.robot as sb

class FollowObject:
    def __init__(self, robot, timer=True):
        self.start_time = time.time()
        print("Starting Follow Object Algorithm...")

        self.robot = robot
        self.base = self.robot.base
        self.timer = timer

        self.moveBy = 0.15
        self.rotBy = 0.15
        self.v = 10.0
        self.a = 5.0
        self.timeout = 1

        self.distance = 0.4
        self.ignore = 0.3

        self.currentState = None
        self.previousState = None
        self.currentStateChanged = True

        self.sub = rospy.Subscriber('/scan', LaserScan, self.computeRegions)
        rospy.spin()

    def computeRegions(self, msg):   
        if self.timer:    
            if time.time() - self.start_time > 30:
                self.robot.stop()
                rospy.signal_shutdown("Ending autonomous mode...") 
        # min_angle = -pi, max_angle = pi, and they both point in front of stretch, where x axis is
        # calculate a difference from min_angle using unit circle, and then use that to get range index
        fleft = int((math.pi/6) / msg.angle_increment)
        left = int((math.pi/2) / msg.angle_increment)
        fright = int((11*math.pi/6) / msg.angle_increment)
        right = int((3*math.pi/2) / msg.angle_increment)

        fleftClosest = min(i for i in msg.ranges[fleft:left] if i > self.ignore)
        frontClosest = min(i for i in msg.ranges[0:fleft] + msg.ranges[fright:] if i > self.ignore)
        frightClosest = min(i for i in msg.ranges[right:fright] if i > self.ignore)

        # Here we want to backup and move away from obstacles when they get too close
        regions = {
        'stop': frontClosest <= 2*self.distance and frontClosest > self.distance,
        'fleft': fleftClosest < 3*self.distance and fleftClosest > 2*self.distance,
        'front':  frontClosest < 3*self.distance and frontClosest > 2*self.distance,
        'fright':  frightClosest < 3*self.distance and frightClosest > 2*self.distance,
        'backup': fleftClosest <= self.distance or frontClosest <= self.distance or frightClosest <= self.distance
        }

        self.takeAction(regions)
        if self.currentStateChanged: # only print the state when it changes
            print("Current: {}, Previous: {}".format(self.currentState, self.previousState))
            self.previousState = self.currentState

    def takeAction(self, regions):
        # Left is positive, right is negative

        xm = 0
        xr = 0
        tempState = self.currentState
        oldDelay = 0.1

        delay = oldDelay # updates delay based on action calculated to allow sufficient time 

        # Single region detected by a normal size object
        if regions['front'] and not regions['fright'] and not regions['fleft']: # Obstacle only in the front
            tempState = 'front'
            xm = self.moveBy
        elif regions['fright'] and not regions['front'] and not regions['fleft']: # Obstacle only on fright
            tempState = 'fright'
            xr = -self.rotBy
        elif regions['fleft'] and not regions['front'] and not regions['fright']: # Obstacle only on fleft
            tempState = 'fleft'
            xr = self.rotBy

        # Multiple regions detected because of large object
        elif regions['front'] and regions['fright'] and not regions['fleft']: # Obstacle only on front and fright
            if self.previousState == 'fright':
                xr = -self.rotBy
            elif self.previousState == 'front':
                xm = self.moveBy
        elif regions['front'] and regions['fleft'] and not regions['fright']: # Obstacle only on front and fleft
            if self.previousState == 'fleft':
                xr = self.rotBy
            elif self.previousState == 'front':
                xm = self.moveBy
        elif regions['fleft'] and regions['fright'] and not regions['front']: # Obstacle only on fright and fleft
            if self.previousState == 'fleft':
                xr = self.rotBy
            elif self.previousState == 'fright':
                xr = -self.rotBy
        elif regions['front'] and regions['fleft'] and regions['fright']: # Obstacle all over the front sections
            if self.previousState == 'fleft':
                xr = self.rotBy
            elif self.previousState == 'fright':
                xr = -self.rotBy
            elif self.previousState == 'front':
                xm = -self.moveBy

        # Too close, backup. Evaluate as a separate case
        if regions['backup']: 
            tempState = 'backup'
            xm = -self.moveBy
            xr = 0

        # Too close, backup. Evaluate as a separate case
        if regions['stop']: 
            tempState = 'stop'
            xm = 0
            xr = 0

        # If state did not change, we do not update previous state. Or else both can become the same and we lose our previous information
        if tempState == self.currentState:
            self.currentStateChanged = False
        else:
            self.currentStateChanged = True

        self.currentState = tempState

        if xm != 0:
            self.move_base(xm)
            self.robot.push_command()
        
        if xr != 0:
            self.rotate_base(xr)
            self.robot.push_command()
            
        time.sleep(delay)

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
    args = argparse.ArgumentParser()
    args.add_argument('--timer', default=False, type=str, help='what avoidance algorithm to run')
    args, unknown = args.parse_known_args()
    timer = int(args.timer)

    r = sb.Robot()
    r.startup()
    rospy.init_node('follow_objects')
    FollowObject(r, timer)