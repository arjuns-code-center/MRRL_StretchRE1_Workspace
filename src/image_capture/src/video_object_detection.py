'''
Author: Arjun Viswanathan
Date created: 8/21/23
Date last modified: 9/14/23
Description: sample computer vision script for object detection off a live camera feed. Usign OpenCV ArUCO marker detection
'''

import cv2

camera = cv2.VideoCapture(0)
success = 1

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

print("Reading from camera...\n")

while success:
    success, image = camera.read()
    s = image.shape

    (corners, ids, rejected) = detector.detectMarkers(image)

    if len(corners) > 0:
        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0,255,0), 2)
            cv2.line(image, topRight, bottomRight, (0,255,0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0,255,0), 2)
            cv2.line(image, bottomLeft, topLeft, (0,255,0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2)
            cY = int((topLeft[1] + bottomRight[1]) / 2)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("Marker detected! ID: {}, Center: {}, Dim: {}".format(str(markerID), [cX, cY], s))

            #rvec, tvec, _ = cv2.aruco.esstimatePoseSingleMarkers(corners, )

    cv2.imshow("ArUCO Detection", image)

    if cv2.waitKey(1) == 27: # ESC key to exit
        break

print("Camera terminated. Finished reading!\n")
cv2.destroyAllWindows()