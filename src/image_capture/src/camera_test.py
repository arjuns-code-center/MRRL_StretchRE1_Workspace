import cv2

camera = cv2.VideoCapture(0)

while True:
    retval, img = camera.read()
    cv2.imshow("camera output", img)

    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()