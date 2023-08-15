from imageai.Detection import ObjectDetection
import os
import cv2
import requests

execution_path = os.getcwd()
retinanet_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/retinanet_resnet50_fpn_coco-eeacb38b.pth'
yolov3_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/yolov3.pt'
tinyyolo_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/tiny-yolov3.pt'

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
custom_objects = detector.CustomObjects(person=True)

camera = cv2.VideoCapture(0)
success = 1

print("Reading from camera...\n")
while success:
    success, image = camera.read()

    detections = detector.detectObjectsFromImage(
                    custom_objects=custom_objects,
                    input_image=image,
                    output_image_path=os.path.join(execution_path, "detected.jpg"),
                    minimum_percentage_probability=40)
    
    for eachObject in detections:
        print(eachObject["name"] , " : ", eachObject["percentage_probability"], " : ", eachObject["box_points"] )
        print("--------------------------------")
print("Camera terminated. Finished reading!\n")