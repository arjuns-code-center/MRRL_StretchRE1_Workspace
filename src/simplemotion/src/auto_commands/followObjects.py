# Author: Arjun Viswanathan
# Date created: 4/13/23
# Last modified date: 9/19/23
# Summary: follows a single object in front of it using computer vision from ArUCO markers

# How to run from command line:
# rosrun simplemotion followObjects.py --timer=0
# For integration with keyboard_teleop, nothing to be done

# Helpful: https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/

# Import system packages

import time
import argparse
import cv2
import numpy as np

# Import ROS specific packages
import rospy
import message_filters
from sensor_msgs.msg import Image
import stretch_body.robot as sb
from cv_bridge import CvBridge, CvBridgeError

class FollowObject:
    def __init__(self, robot, timer=True):
        self.start_time = time.time()
        print("Starting Follow Object Algorithm...")

        self.robot = robot
        self.base = self.robot.base
        self.timer = timer

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.moveBy = 0.15
        self.rotBy = 0.15
        self.v = 10.0
        self.a = 5.0
        self.timeout = 1

        self.distance = 0.4
        self.ignore = 0.3

        self.rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.sync = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.sync.registerCallback(self.takeAction)
        rospy.spin()

    def takeAction(self, rgb_img, depth_img):
        if self.timer:
            if (time.time() - self.start_time) > 30:
                self.robot.stop()
                rospy.signal_shutdown("Ending autonomous mode...")
                
        xm = 0
        xr = 0
        mp = (0, 0)
        depth = 0

        try:
            cv2_rgbimg = CvBridge().imgmsg_to_cv2(rgb_img, 'bgr8')
            cv2_depthimg = CvBridge().imgmsg_to_cv2(depth_img, depth_img.encoding)

            if cv2_rgbimg is not None:
                s = cv2_rgbimg.shape # (height, width, channels)

                (corners, ids, rejected) = self.detector.detectMarkers(cv2_rgbimg)

                if len(corners) > 0:
                    ids = ids.flatten()

                    for (markerCorner, markerID) in zip(corners, ids):
                        corners = markerCorner.reshape((4,2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners

                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))

                        cv2.line(cv2_rgbimg, topLeft, topRight, (0,255,0), 2)
                        cv2.line(cv2_rgbimg, topRight, bottomRight, (0,255,0), 2)
                        cv2.line(cv2_rgbimg, bottomRight, bottomLeft, (0,255,0), 2)
                        cv2.line(cv2_rgbimg, bottomLeft, topLeft, (0,255,0), 2)

                        mp[0] = int((topLeft[0] + bottomRight[0]) / 2)
                        mp[1] = int((topLeft[1] + bottomRight[1]) / 2)
                        cv2.circle(cv2_rgbimg, (mp[0], mp[1]), 4, (0, 0, 255), -1)

                        cv2.putText(cv2_rgbimg, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        if cv2_depthimg is not None:
                            depth = cv2_depthimg[mp[0], mp[1]]

                        print("Marker Detected! ID: {}, Center: {}, Distance: {}".format(str(markerID), mp, depth))

                if depth > self.ignore:
                    if depth > self.distance:
                        xm = self.moveBy
                    elif depth < self.distance:
                        xm = -self.moveBy

                if mp[1] < s[1] / 2:
                    xr = -self.rotBy
                elif mp[1] > s[1] / 2:
                    xr = self.rotBy
            else:
                print("No marker detected :(")
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge Error: {0}'.format(e))

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
    args = argparse.ArgumentParser()
    args.add_argument('--timer', default=False, type=str, help='what avoidance algorithm to run')
    args, unknown = args.parse_known_args()
    timer = int(args.timer)

    r = sb.Robot()
    r.startup()
    rospy.init_node('follow_objects')
    FollowObject(r, timer)