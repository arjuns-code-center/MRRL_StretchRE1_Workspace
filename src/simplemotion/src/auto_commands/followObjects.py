# Author: Arjun Viswanathan
# Date created: 4/13/23
# Last modified date: 8/28/23
# Summary: follows a single object in front of it using computer vision from ImageAI library

# How to run from command line:
# rosrun simplemotion followObjects.py --timer=0
# For integration with keyboard_teleop, nothing to be done

# Helpful: https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/

# Import system packages

# TODO: 
# Find the ROI for robot. Anything outside this box does not matter
# If min value from depth map is inside box, then find value
# If value < threshold, then take action to avoid
# Else, keep moving forward (in both cases)

import time
import argparse
import cv2
import numpy as np

from imageai.Detection import ObjectDetection
import os
import requests

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

        self.execution_path = os.getcwd()
        self.retinanet_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/retinanet_resnet50_fpn_coco-eeacb38b.pth'
        self.yolov3_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/yolov3.pt'
        self.tinyyolo_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/tiny-yolov3.pt'

        self.detector = ObjectDetection()
        self.detector.setModelTypeAsYOLOv3()

        # r = requests.get(retinanet_url, allow_redirects=True)
        r = requests.get(self.yolov3_url, allow_redirects=True)
        # r = requests.get(tinyyolo_url, allow_redirects=True)

        # open('models/retinanet_resnet50_fpn_coco-eeacb38b.pth', 'wb').write(r.content)
        open('models/yolov3.pt', 'wb').write(r.content)
        # open('models/tiny-yolov3.pt', 'wb').write(r.content)

        # detector.setModelPath(os.path.join(execution_path , "models/retinanet_resnet50_fpn_coco-eeacb38b.pth"))
        self.detector.setModelPath(os.path.join(self.execution_path , "models/yolov3.pt"))
        # detector.setModelPath(os.path.join(execution_path , "models/tiny-yolov3.pt"))

        self.detector.loadModel()

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

        self.rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image', Image)
        self.sync = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.sync.registerCallback(self.takeAction)
        rospy.spin()

    def takeAction(self, rgb_img, depth_img):
        xm = 0
        xr = 0
        mp = (0, 0)

        try:
            cv2_rgbimg = CvBridge().imgmsg_to_cv2(rgb_img, 'brg8')
            cv2_depthimg = CvBridge().imgmsg_to_cv2(depth_img, 'brg8')
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge Error: {0}'.format(e))

        if cv2_rgbimg is not None:
            detections = self.detector.detectObjectsFromImage(
                            input_image=cv2_rgbimg,
                            output_image_path=os.path.join(self.execution_path, "detected.jpg"),
                            minimum_percentage_probability=60)
            
            out_img = cv2.imread(os.path.join(self.execution_path, "detected.jpg"))
            cv2.imshow("output", out_img)
            
            for eachObject in detections:
                if eachObject["name"] == "person":
                    bp = detections[0]["box points"] # (x1, y1, x2, y2)
                    bp_cl = bp[2] - bp[0]
                    bp_rl = bp[3] - bp[1]

                    if cv2_depthimg is not None:
                        roi = cv2_depthimg[bp[1]:bp[1] + bp_rl, bp[0]:bp[0] + bp_cl]
                        depth = np.min(roi)

                    s = out_img.shape # (height, width, 3)
                    break

        if depth > self.ignore and depth > self.distance:
            xm = self.moveBy
        elif depth > self.ignore and depth < self.distance:
            xm = -self.moveBy

        if mp[0] < s[0]:
            xr = self.rotBy
        elif mp[0] > s[0]:
            xr = -self.rotBy

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