# Author: Arjun Viswanathan
# Date created: 4/13/23
# Last modified date: 10/2/23
# Summary: follows a single ArUco marker in front of it using computer vision

# How to run from command line:
# rosrun simplemotion followMarker.py
# rosrun simplemotion followMarker.py --timer=<TIME>
# For integration with keyboard_teleop, nothing to be done

# Helpful: https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/

# Import system packages

import time
import argparse
import cv2
import scipy.io as sio
import numpy as np

# Import ROS specific packages
import rospy
from sensor_msgs.msg import Image
import stretch_body.robot as sb
from cv_bridge import CvBridge, CvBridgeError

class FollowMarker:
    def __init__(self, robot, timer=False):
        self.start_time = time.time()
        print("Starting Follow Object Algorithm...")

        self.robot = robot
        self.base = self.robot.base
        self.timer = timer

        # Load camera parameters from MATLAB
        self.path = "~/motion_ws/src/image_capture/src/calibration/"

        # camParams = sio.loadmat(self.path + "d435i_camParams.mat")
        self.cameraMatrix = np.float32([[295.6886, 0, 321.6002], [0, 297.0698, 229.0389], [0, 0, 1]])
        self.distCoeffs = np.float32([-0.0606, 0.0274, 0, 0, 0])

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.markerLength = 100 # mm

        self.moveBy = 0.15
        self.rotBy = 0.15
        self.v = 10.0
        self.a = 5.0
        self.timeout = 1

        self.distance = 500 # mm
        self.ignore = 300

        self.j = 0

        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.takeAction)
        rospy.spin()

    def takeAction(self, rgb_img):
        if self.timer:
            if (time.time() - self.start_time) > 30:
                self.robot.stop()
                rospy.signal_shutdown("Ending autonomous mode...")
                
        xm = 0
        xr = 0
        mp = [0, 0]
        depth = 0

        try:
            cv2_rgbimg = CvBridge().imgmsg_to_cv2(rgb_img, 'bgr8')

            if cv2_rgbimg is not None:
                s = cv2_rgbimg.shape # (height, width, channels)

                (corners, ids, rejected) = self.detector.detectMarkers(cv2_rgbimg)

                if len(corners) > 0:
                    ids = ids.flatten()

                    # For every detected marker, we do pose estimation using its corners and find the rotational and translational vectors
                    for (markerCorner, markerID) in zip(corners, ids):
                        reshapedCorners = markerCorner.reshape((4, 2))
                        (tL, tR, bR, bL) = reshapedCorners
                        tL = [int(tL[0]), int(tL[1])]
                        tR = [int(tR[0]), int(tR[1])]
                        bR = [int(bR[0]), int(bR[1])]
                        bL = [int(bL[0]), int(bL[1])]

                        # mp[0] = int((tL[0] + bR[0]) / 2)
                        # mp[1] = int((tL[1] + bR[1]) / 2)
                        
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.markerLength, self.cameraMatrix, self.distCoeffs)
                        rvec = rvec[0][0]
                        tvec = tvec[0][0]

                        mp[0] = tvec[0]
                        mp[1] = tvec[1]

                        depth = round(tvec[2], 2) # mm

                        # Printing distance on the image
                        cv2.putText(cv2_rgbimg, str(depth), (tL[0], tL[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        print("Marker detected! ID: {}, Center: {}, Distance (mm): {}".format(str(markerID), mp, depth))

                    # Press 's' key when detecting marker to save image. Only available when marker is detected
                    if cv2.waitKey(33) == ord('s'):
                        print("Taking ArUco pic {}...".format(j))
                        cv2.imwrite(self.path + "Images/aruco_image_{}.png".format(self.j), cv2_rgbimg)
                        self.j += 1
            else:
                print("No marker detected :(")
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge Error: {0}'.format(e))

        if depth > self.ignore:
            if depth > self.distance:
                xm = self.moveBy
            elif depth < self.distance:
                xm = -self.moveBy

        # If it detected a marker, mp != 0
        # if mp[0] != 0 and mp[1] != 0:
        #     if mp[0] < 0:
        #         xr = self.rotBy
        #     elif mp[0] > 0:
        #         xr = -self.rotBy

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
    rospy.init_node('follow_marker')
    FollowMarker(r, timer)