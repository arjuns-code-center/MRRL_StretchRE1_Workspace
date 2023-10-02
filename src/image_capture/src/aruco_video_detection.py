'''
Author: Arjun Viswanathan
SDP Team 12 ArUco Detection Script
Date created: 9/25/23
Date last modified: 10/2/23
Description: base code for detecting ArUco markers off a live camera feed
'''

import cv2
import scipy.io as sio
import pyrealsense2 as rs
import numpy as np

camtype = 'realsense'

# Load camera parameters from MATLAB
path = "E:/Research/UMass_MRRL/MRRL_StretchRE1_Workspace/src/image_capture/src/calibration/"
path_stretch = "~/motion_ws/src/image_capture/src/calibration/"

# camParams = sio.loadmat(path + "arjunPC_camParams.mat")
# camParams = sio.loadmat(path + "arjunLaptop_camParams.mat")
# camParams = sio.loadmat(path + "d435i_camParams.mat")
camParams = sio.loadmat(path + "d455i_camParams.mat")
cameraMatrix = camParams['cameraMatrix']
distCoeffs = camParams['distortionCoefficients']

# Set resolution and frame size when displaying
if camtype == 'cv2':
    camera = cv2.VideoCapture(0)
    camera.set(3, 1920)
    camera.set(4, 1080)
elif camtype == 'realsense':
    pipe = rs.pipeline()
    cfg = rs.config()

    w = 640
    h = 480

    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, 30)

    pipe.start(cfg)

# Set up the ArUco dictionary and detector object
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
markerLength = 25 # mm

print("Reading from camera...\n")

# To keep track of saved images
i = 0

while True:
    if camtype == 'cv2':
        success, image = camera.read()
    elif camtype == 'realsense':
        frame = pipe.wait_for_frames()
        # depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()

        # depth_img = np.asanyarray(depth_frame.get_data())
        image = np.asanyarray(color_frame.get_data())
        # depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.5), cv2.COLORMAP_JET)

    s = image.shape

    # First we detect all markers in the frame
    (corners, ids, rejected) = detector.detectMarkers(image)
 
    if len(corners) > 0: # we have detected something
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(image, corners, ids) # draw outlines for all 

        # For every detected marker, we do pose estimation using its corners and find the rotational and translational vectors
        for (markerCorner, markerID) in zip(corners, ids):
            reshapedCorners = markerCorner.reshape((4, 2))
            (tL, tR, bR, bL) = reshapedCorners
            topLeft = [int(tL[0]), int(tL[1])]
            
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, markerLength, cameraMatrix, distCoeffs)
            rvec = rvec[0][0]
            tvec = tvec[0][0]

            # Printing distance on the image
            cv2.putText(image, str(round(tvec[2], 2)), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("Marker detected! ID: {}, RVEC: {}, TVEC: {}".format(str(markerID), rvec, tvec))

        # Press 'a' key when detecting marker to save image. Only available when marker is detected
        if cv2.waitKey(33) == ord('a'):
            print("Taking ArUco pic {}...".format(i))
            cv2.imwrite(path + "Images/aruco_image_{}.png".format(i), image)
            i += 1

    cv2.imshow("ArUco Detection", image)

    if cv2.waitKey(1) == 27: # ESC key to exit
        break

print("Camera terminated. Finished reading!\n")
cv2.destroyAllWindows()