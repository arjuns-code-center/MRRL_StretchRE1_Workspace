from imageai.Detection import VideoObjectDetection
import os
import cv2
import requests

execution_path = os.getcwd()
retinanet_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/retinanet_resnet50_fpn_coco-eeacb38b.pth'
yolov3_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/yolov3.pt'
tinyyolo_url = 'https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/tiny-yolov3.pt'

camera = cv2.VideoCapture(0)

detector = VideoObjectDetection()
detector.setModelTypeAsRetinaNet()

r = requests.get(retinanet_url, allow_redirects=True)
# r = requests.get(yolov3_url, allow_redirects=True)
# r = requests.get(tinyyolo_url, allow_redirects=True)

open('models/retinanet_resnet50_fpn_coco-eeacb38b.pth', 'wb').write(r.content)
# open('models/yolov3.pt', 'wb').write(r.content)
# open('models/tiny-yolov3.pt', 'wb').write(r.content)

detector.setModelPath(os.path.join(execution_path , "models/retinanet_resnet50_fpn_coco-eeacb38b.pth"))
# detector.setModelPath(os.path.join(execution_path , "models/yolov3.pt"))
# detector.setModelPath(os.path.join(execution_path , "models/tiny-yolov3.pt"))

detector.loadModel()

video_path = detector.detectObjectsFromVideo(
                camera_input=camera,
                output_file_path=os.path.join(execution_path, "camera_detected_video"),
                frames_per_second=20, log_progress=True, minimum_percentage_probability=40)