#!/usr/bin/env python
'''
Image capture service which uses webcam to capture and save an image
Captured image is saved in the Images folder in the src directory of package

Author: Arjun Viswanathan
Date created: 9/27/22

On robot:
Port 6: External camera 2
'''

import cv2

def capture_and_save():
    identifier = ".png"

    cam = cv2.VideoCapture(6)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)

    result, image = cam.read()

    if result:
        print("Captured an image!")
        name = "myimage"

        name = input("Enter name for image: ")
        mrn = input("Enter Medical Record Number (MRN): ")
        loc = input("Enter body location: ")
        distancefromcam = input("Enter distance from camera with metric: ")
        length = input("Enter feature length with metric: ")
        width = input("Enter feature width with metric: ")

        cv2.detailEnhance(image, sigma_s=10, sigma_r=0.15) # enhance image detail

        if not cv2.imwrite("images/{}{}".format(name, identifier), image):
            raise Exception("Could not save image")

        print("Saved {} at ~/motion_ws/images".format(name))

        imagedict = open('images/Image_Dictionary.txt', 'a')
        imagedict.write("\n{} | {} | {} | {} | {} x {}".format(name, mrn, loc, distancefromcam, length, width))
        imagedict.close()
    else:
        print("No image detected. Please try again")

    return 0

def image_capture():
    print("Ready to capture images")
    res = capture_and_save()
    return res

if __name__ == "__main__":
    image_capture()
