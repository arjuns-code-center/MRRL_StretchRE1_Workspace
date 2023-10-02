'''
Author: Arjun Viswanathan
Date created: 9/29/23
Date last modified: 9/29/23
Description: sample script to read RGB and depth images from Intel RealSense d435i camera at 30 FPS
'''

import pyrealsense2 as rs
import numpy as np
import cv2

path = "E:/Research/UMass_MRRL/MRRL_StretchRE1_Workspace/src/image_capture/src/calibration/"
path_stretch = "~/motion_ws/src/image_capture/src/calibration/"

pipe = rs.pipeline()
cfg = rs.config()

w = 640
h = 480

cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, 30)

pipe.start(cfg)

i = 0

while True:
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    depth_img = np.asanyarray(depth_frame.get_data())
    color_img = np.asanyarray(color_frame.get_data())
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.5), cv2.COLORMAP_JET)
    
    cv2.imshow('rgb', color_img)
    cv2.imshow('depth', depth_cm)

    # Press 'a' key to save image for calibration
    if cv2.waitKey(33) == ord('a'):
        print("Taking pic {}...".format(i))
        cv2.imwrite(path + "Images/image_{}.png".format(i), color_img)
        i += 1

    if cv2.waitKey(1) == 27:
        break

pipe.stop()
cv2.destroyAllWindows()