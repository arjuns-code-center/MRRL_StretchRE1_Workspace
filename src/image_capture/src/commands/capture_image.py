# Author: Arjun Viswanathan
# Date created: 9/27/22
# Last date modified: 5/4/23
#
# Image capture service which uses webcam to capture and save an image
# Captured image is saved in the Images folder in the src directory of package

import cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os, sys, termios, tty

class ImageCapture:
    def __init__(self, count=0):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.capture_and_save)
        self.save_path = '/home/arjun/motion_ws/src/image_capture/src/calibration/Images/'
        self.count = count

        print("Starting node...")
        rospy.spin()

    def getKeystroke(self):
        fd=sys.stdin.fileno()
        old_settings=termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())
            ch=sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)

        return ch

    def capture_and_save(self, msg):
        key = self.getKeystroke()

        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge Error: {0}'.format(e))

        if image is not None:
            name = 'image_' + str(self.count) + '.png'
            path = os.path.join(self.save_path, name)

            if key == 'a':
                cv2.imwrite(path, image)
                print("Saved {}".format(name))
                self.count += 1
        else:
            print("No image detected. Please try again")

if __name__ == "__main__":
    rospy.init_node("capture_image")
    imc = ImageCapture(0)