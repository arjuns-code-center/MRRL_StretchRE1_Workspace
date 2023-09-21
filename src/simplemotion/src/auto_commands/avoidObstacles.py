# Author: Arjun Viswanathan
# Date created: 3/9/23
# Last modified date: 9/20/23
# Summary: Navigate around obstacles in front of stretch using LiDAR in autonomous mode
# SimpleAvoid: performs avoidance while continuously going forward
# BetterAvoid: performs avoidance and considers previous states to navigate better
# BetterAvoidWithGoal: performs avoidance with previous states but with another added constraint of minimizing distance to goal

# How to run the file from command line:
# rosrun simplemotion avoidObstacles.py --algotype=<SPECIFY TYPE> --timer=0 --goal=<SPECIFY GOAL TUPLE>
# For integration with keyboard_teleop, it will default to BetterAvoid

# Import system packages
import math
import time
import argparse
import numpy as np
import math

# Import ROS specific packages
import rospy
from sensor_msgs.msg import LaserScan
import stretch_body.robot as sb

class SimpleAvoid:
    def __init__(self, robot, timer=True):
        self.start_time = time.time()
        print("Starting Simple Navigation Algorithm...")

        self.robot = robot
        self.base = self.robot.base
        self.timer = timer

        self.v = 10.0 #self.base.params['motion']['max']['vel_m']
        self.a = 5.0 #self.base.params['motion']['max']['accel_m']
        self.timeout = 1

        self.distance = 1.0
        self.ignore = 0.25

        self.sub = rospy.Subscriber('/scan', LaserScan, self.computeRegions)
        rospy.spin()

    def computeRegions(self, msg):
        if self.timer:
            if (time.time() - self.start_time) > 30:
                self.robot.stop()
                rospy.signal_shutdown("Ending autonomous mode...")

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
    def __init__(self, robot, timer=True):
        self.start_time = time.time()
        print("Starting Better Navigation Algorithm...")

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

        self.sub = rospy.Subscriber('/scan', LaserScan, self.computeBetterRegions)
        rospy.spin()

    def computeBetterRegions(self, msg):
        if self.timer:
            if (time.time() - self.start_time) > 30:
                self.robot.stop()
                rospy.signal_shutdown("Ending autonomous mode...")

        # min_angle = -pi, max_angle = pi, and they both point in front of stretch, where x axis is
        # calculate a difference from min_angle using unit circle, and then use that to get range index
        fleft = int((math.pi/6) / msg.angle_increment)
        left = int((math.pi/3) / msg.angle_increment)
        fright = int((11*math.pi/6) / msg.angle_increment)
        right = int((5*math.pi/3) / msg.angle_increment)
        b1 = int((5*math.pi/6) / msg.angle_increment)
        b2 = int((7*math.pi/6) / msg.angle_increment)

        fleftClosest = min(i for i in msg.ranges[fleft:left] if i > self.ignore)
        frontClosest = min(i for i in msg.ranges[0:fleft] + msg.ranges[fright:] if i > self.ignore)
        frightClosest = min(i for i in msg.ranges[right:fright] if i > self.ignore)
        backClosest = min(i for i in msg.ranges[b1:b2] if i > self.ignore)

        # Here we want to backup and move away from obstacles when they get too close
        regions = {
        'backup': fleftClosest <= self.distance or frontClosest <= self.distance or frightClosest <= self.distance,
        'gofront': backClosest <= self.distance + 0.15,
        'fleft': fleftClosest <= 2*self.distance and fleftClosest > self.distance,
        'front':  frontClosest <= 2*self.distance and frontClosest > self.distance,
        'fright':  frightClosest <= 2*self.distance and frightClosest > self.distance
        }

        self.takeBetterAction(regions)
        if self.currentStateChanged: # only print the state when it changes
            print("Current: {}, Previous: {}".format(self.currentState, self.previousState))
            self.previousState = self.currentState

    def takeBetterAction(self, regions):
        # Keep track of a previous state so we do not deadlock between 2 actions forever
        # Stretch can detect an obstacle in fright, turn left to avoid, and detect in front, turn right to avoid, and continue doing this forever
        # Based on the previous state, actions will change
        # Left is positive, right is negative

        xm = 0
        xr = 0
        tempState = self.currentState
        oldDelay = 0.1
        newDelay = 0.2

        delay = oldDelay # updates delay based on action calculated to allow sufficient time 

        # Single region detected by a normal size object
        if regions['front'] and not regions['fright'] and not regions['fleft']: # Obstacle only in the front
            tempState = 'front'

            # Need to consider both sides 
            if self.previousState == 'fright' or self.previousState == 'front and fright':
                xr = self.rotBy * 2
                delay = newDelay
            elif self.previousState == 'fleft' or self.previousState == 'front and fleft':
                xr = -self.rotBy * 2
                delay = newDelay
            else:
                xr = -self.rotBy # default action is to turn right
        elif regions['fright'] and not regions['front'] and not regions['fleft']: # Obstacle only on fright
            tempState = 'fright'

            # Only need to consider if something was in front since you turn left anyways
            if self.previousState == 'front' or self.previousState == 'front and fleft':
                xr = -self.rotBy * 2
                delay = newDelay
            else:
                xr = self.rotBy
        elif regions['fleft'] and not regions['front'] and not regions['fright']: # Obstacle only on fleft
            tempState = 'fleft'

            # Only need to consider if something was in front since you turn right anyways
            if self.previousState == 'front' or self.previousState == 'front and fright':
                xr = self.rotBy * 2
                delay = newDelay
            else:
                xr = -self.rotBy

        # Multiple regions detected because of large object
        elif regions['front'] and regions['fright'] and not regions['fleft']: # Obstacle only on front and fright
            tempState = 'front and fright'

            if self.previousState == 'fleft' or self.previousState == 'front and fleft' or self.previousState == 'front':
                xr = -self.rotBy * 2
                delay = newDelay
            else:
                xr = self.rotBy
        elif regions['front'] and regions['fleft'] and not regions['fright']: # Obstacle only on front and fleft
            tempState = 'front and fleft'

            if self.previousState == 'fright' or self.previousState == 'front and fright' or self.previousState == 'front':
                xr = self.rotBy * 2
                delay = newDelay
            else:
                xr = -self.rotBy
        elif regions['fleft'] and regions['fright'] and not regions['front']: # Obstacle only on fright and fleft
            tempState = 'fleft and fright'
            xm = self.moveBy
        elif regions['front'] and regions['fleft'] and regions['fright']: # Obstacle all over the front sections
            tempState = 'all'
            xm = -self.moveBy
        elif regions['gofront']:
            tempState = 'gofront'

            if self.previousState == 'backup':
                xr = -1.57
                delay = newDelay
            else:
                xm = self.moveBy
        else:
            tempState = 'nothing'
            xm = self.moveBy

        # Too close, backup. Evaluate as a separate case
        if regions['backup']: 
            tempState = 'backup'
            xm = -self.moveBy
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

class BetterAvoidWithGoal:
    def __init__(self, robot, startCoords=(0, 0), goalCoords=(0, 0)):
        self.start_time = time.time()
        print("Starting Better Navigation Algorithm With Goal...")

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

        self.startCoords = startCoords
        self.goalCoords = goalCoords

        self.possibleActions = {
            'left': (-1, 0),
            'front': (0, 1),
            'right': (1, 0),
            'back': (0, -1)
        }

        self.movements = {
            'left': self.rotBy * 2,
            'front': self.moveBy,
            'right': -self.rotBy * 2,
            'back': -self.moveBy
        }

        self.sub = rospy.Subscriber('/scan', LaserScan, self.computeBetterRegions)
        rospy.spin()

    def computeBetterRegions(self, msg):
        if self.startCoords == self.goalCoords:
            self.robot.stop()
            rospy.signal_shutdown("Goal reached! Ending autonomous mode...")

        # min_angle = -pi, max_angle = pi, and they both point in front of stretch, where x axis is
        # calculate a difference from min_angle using unit circle, and then use that to get range index
        fleft = int((math.pi/6) / msg.angle_increment)
        left = int((math.pi/3) / msg.angle_increment)
        fright = int((11*math.pi/6) / msg.angle_increment)
        right = int((5*math.pi/3) / msg.angle_increment)
        b1 = int((5*math.pi/6) / msg.angle_increment)
        b2 = int((7*math.pi/6) / msg.angle_increment)

        fleftClosest = min(i for i in msg.ranges[fleft:left] if i > self.ignore)
        frontClosest = min(i for i in msg.ranges[0:fleft] + msg.ranges[fright:] if i > self.ignore)
        frightClosest = min(i for i in msg.ranges[right:fright] if i > self.ignore)
        backClosest = min(i for i in msg.ranges[b1:b2] if i > self.ignore)

        # Here we want to backup and move away from obstacles when they get too close
        regions = {
        'backup': fleftClosest <= self.distance or frontClosest <= self.distance or frightClosest <= self.distance,
        'gofront': backClosest <= self.distance + 0.15,
        'fleft': fleftClosest <= 2*self.distance and fleftClosest > self.distance,
        'front':  frontClosest <= 2*self.distance and frontClosest > self.distance,
        'fright':  frightClosest <= 2*self.distance and frightClosest > self.distance
        }

        self.takeBetterAction(regions)
        if self.currentStateChanged: # only print the state when it changes
            print("Current: {}, Previous: {}".format(self.currentState, self.previousState))
            self.previousState = self.currentState

    def takeBetterAction(self, regions):
        # Keep track of a previous state so we do not deadlock between 2 actions forever
        # Stretch can detect an obstacle in fright, turn left to avoid, and detect in front, turn right to avoid, and continue doing this forever
        # Based on the previous state, actions will change
        # Left is positive, right is negative

        xm = 0
        xr = 0
        tempState = self.currentState
        oldDelay = 0.1
        newDelay = 0.2

        delay = oldDelay # updates delay based on action calculated to allow sufficient time 

        d = {
            'left': 0,
            'right': 0,
            'front': 0,
            'back': 0
        }
        for action in self.possibleActions:
            d[action] = self.calculateManhattanDistance(self.startCoords + action)

        o = []
        # Figure out where the obstacles are at according to LiDAR detected regions
        # Single region detected by a normal size object
        if regions['front'] and not regions['fright'] and not regions['fleft']: # Obstacle only in the front
            tempState = 'front'
            o.append('front')

            # Need to consider both sides 
            if self.previousState == 'fright' or self.previousState == 'front and fright':
                # xr = self.rotBy * 2
                o.append('right')
                delay = newDelay
            elif self.previousState == 'fleft' or self.previousState == 'front and fleft':
                # xr = -self.rotBy * 2
                o.append('left')
                delay = newDelay
            # else:
                # xr = -self.rotBy # default action is to turn right
        elif regions['fright'] and not regions['front'] and not regions['fleft']: # Obstacle only on fright
            tempState = 'fright'
            o.append('right')

            # Only need to consider if something was in front since you turn left anyways
            if self.previousState == 'front' or self.previousState == 'front and fleft':
                # xr = -self.rotBy * 2
                o.remove('right')
                o.append('left')
                delay = newDelay
            # else:
            #     xr = self.rotBy
        elif regions['fleft'] and not regions['front'] and not regions['fright']: # Obstacle only on fleft
            tempState = 'fleft'
            o.append('left')

            # Only need to consider if something was in front since you turn right anyways
            if self.previousState == 'front' or self.previousState == 'front and fright':
                # xr = self.rotBy * 2
                o.remove('left')
                o.append('right')
                delay = newDelay
            # else:
            #     xr = -self.rotBy

        # Multiple regions detected because of large object
        elif regions['front'] and regions['fright'] and not regions['fleft']: # Obstacle only on front and fright
            tempState = 'front and fright'
            o.append('front')
            o.append('right')

            if self.previousState == 'fleft' or self.previousState == 'front and fleft' or self.previousState == 'front':
                # xr = -self.rotBy * 2
                o.remove('right')
                o.append('left')
                delay = newDelay
            # else:
            #     xr = self.rotBy
        elif regions['front'] and regions['fleft'] and not regions['fright']: # Obstacle only on front and fleft
            tempState = 'front and fleft'
            o.append('front')
            o.append('left')

            if self.previousState == 'fright' or self.previousState == 'front and fright' or self.previousState == 'front':
                # xr = self.rotBy * 2
                o.remove('left')
                o.append('right')
                delay = newDelay
            # else:
            #     xr = -self.rotBy
        elif regions['fleft'] and regions['fright'] and not regions['front']: # Obstacle only on fright and fleft
            tempState = 'fleft and fright'
            o.append('left')
            o.append('right')
            # xm = self.moveBy
        elif regions['front'] and regions['fleft'] and regions['fright']: # Obstacle all over the front sections
            tempState = 'all'
            o.append('front')
            o.append('left')
            o.append('right')
            # xm = -self.moveBy
        elif regions['gofront']:
            tempState = 'gofront'
            o.append('back')

            if self.previousState == 'backup':
                # xr = -1.57
                o.append('front')
                delay = newDelay
            # else:
            #     xm = self.moveBy
        else:
            tempState = 'nothing'
            # xm = self.moveBy

        # Too close, backup. Evaluate as a separate case
        if regions['backup']: 
            tempState = 'backup'
            o.append('front')
            # xm = -self.moveBy
            # xr = 0

        # If state did not change, we do not update previous state. Or else both can become the same and we lose our previous information
        if tempState == self.currentState:
            self.currentStateChanged = False
        else:
            self.currentStateChanged = True

        self.currentState = tempState

        actionToTake = list(d.keys())[list(d.values()).index(min(d[a] for a in self.possibleActions))]
        self.startCoords += self.possibleActions[actionToTake]

        if actionToTake == 'left' or actionToTake == 'right':
            xr = self.movements[actionToTake]
        else:
            xm = self.movements[actionToTake]

        if xm != 0:
            self.move_base(xm)
            self.robot.push_command()
        
        if xr != 0:
            self.rotate_base(xr)
            self.robot.push_command()
            
        time.sleep(delay)

    def calculateManhattanDistance(self, currentCoord):
        return abs(self.goalCoords[0] - currentCoord[0]) + abs(self.goalCoords[1] - currentCoord[1])
    
    def calculateEuclideanDistance(self, currentCoord):
        return math.sqrt((self.goalCoords[0] - currentCoord[0])**2 + (self.goalCoords[1] - currentCoord[1])**2)

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
    args.add_argument('--algotype', default='better', type=str, help='what avoidance algorithm to run')
    args.add_argument('--timer', default=False, type=str, help='what avoidance algorithm to run')
    args.add_argument('--start', default=(0, 0), type=tuple, help='what are the start coordinates')
    args.add_argument('--goal', default=(0, 0), type=tuple, help='what are the goal coordinates')
    args, unknown = args.parse_known_args()
    algorithmType = args.algotype
    timer = int(args.timer)
    start = args.start
    goal = args.goal

    r = sb.Robot()
    r.startup()

    rospy.init_node('avoid_obstacles')

    if algorithmType == 'simple':
        print("Using SimpleAvoid algorithm to navigate obstacles")
        SimpleAvoid(r, timer)
    elif algorithmType == 'better':
        print("Using BetterAvoid algorithm to navigate obstacles")
        BetterAvoid(r, timer)
    elif algorithmType == 'bettergoal':
        print('Using BetterAvoidWithGoal algorithm to navigate obstacles')
        BetterAvoidWithGoal(r, start, goal)