# Author: Arjun Viswanathan
# Date created: 9/27/22
# Last date modified: 5/4/23
#
# Image capture service which uses webcam to capture and save an image
# Captured image is saved in the Images folder in the src directory of package

import cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class ImageCapture:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.capture_and_save, queue_size=1)
        self.save_path = '/home/arjun/motion_ws/src/image_capture/src/Images/'

    def capture_and_save(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge Error: {0}'.format(e))

        if image is not None:
            print("Captured an image!")
            name = 'captured_image.jpg'
            path = os.path.join(self.save_path, name)

            if not cv2.imwrite(path, image):
                print("Did not save image")

            print("Saved {} at ~/motion_ws/src/image_capture/src/Images".format(name))
        else:
            print("No image detected. Please try again")

        rospy.signal_shutdown("done")

if __name__ == "__main__":
    imc = ImageCapture()