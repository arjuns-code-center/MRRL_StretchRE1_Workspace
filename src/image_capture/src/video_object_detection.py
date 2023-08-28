'''
Author: Arjun Viswanathan
Date created: 8/21/23
Date last modified: 8/28/23
Description: sample computer vision script for object detection off a live camera feed. Using open source ImageAI library
https://github.com/OlafenwaMoses/ImageAI/tree/master
'''

from imageai.Detection import ObjectDetection
from PIL import Image
from transformers import pipeline
import os
import cv2
import requests
import numpy as np
import math

execution_path = os.getcwd()
retinanet_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/retinanet_resnet50_fpn_coco-eeacb38b.pth'
yolov3_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/yolov3.pt'
tinyyolo_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/tiny-yolov3.pt'

checkpoint = "vinvino02/glpn-nyu"
depth_estimator = pipeline("depth-estimation", model=checkpoint)

detector = ObjectDetection()
detector.setModelTypeAsYOLOv3()

# r = requests.get(retinanet_url, allow_redirects=True)
r = requests.get(yolov3_url, allow_redirects=True)
# r = requests.get(tinyyolo_url, allow_redirects=True)

# open('models/retinanet_resnet50_fpn_coco-eeacb38b.pth', 'wb').write(r.content)
open('models/yolov3.pt', 'wb').write(r.content)
# open('models/tiny-yolov3.pt', 'wb').write(r.content)

# detector.setModelPath(os.path.join(execution_path , "models/retinanet_resnet50_fpn_coco-eeacb38b.pth"))
detector.setModelPath(os.path.join(execution_path , "models/yolov3.pt"))
# detector.setModelPath(os.path.join(execution_path , "models/tiny-yolov3.pt"))

detector.loadModel()

camera = cv2.VideoCapture(0)
success = 1

print("Reading from camera...\n")

while success:
    success, image = camera.read()
    
    detections = detector.detectObjectsFromImage(
                    input_image=image,
                    output_image_path=os.path.join(execution_path, "detected.jpg"),
                    minimum_percentage_probability=60)
    
    depth_map = np.array(depth_estimator(Image.fromarray(image))["depth"])

    out_img = cv2.imread(os.path.join(execution_path, "detected.jpg"))
    cv2.imshow("output", out_img)
    cv2.imshow("depth map", depth_map)

    for eachObject in detections:
        if eachObject["name"] == "person":
            bp = eachObject["box_points"]
            bp_cl = bp[2] - bp[0]
            bp_rl = bp[3] - bp[1]
            roi = depth_map[bp[1]:bp[1] + bp_rl, bp[0]:bp[0] + bp_cl]

            try:
                d = np.min(roi)
            except IndexError as i:
                d = math.nan
                # continue
            
            print("Boxpoints: {}, Distance: {}".format(bp, d))
            print("--------------------------------")

    if cv2.waitKey(1) == 27: # ESC key to exit
        break

print("Camera terminated. Finished reading!\n")
cv2.destroyAllWindows()

os.remove('models/yolov3.pt')
# os.remove('models/tiny-yolov3.pt')
# os.remove('models/retinanet_resnet50_fpn_coco-eeacb38b.pth')