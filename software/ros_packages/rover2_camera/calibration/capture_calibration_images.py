# Source - https://stackoverflow.com/a/34588758
# Posted by derricw, modified by community. See post 'Timeline' for change history
# Retrieved 2026-04-07, License - CC BY-SA 4.0

import cv2
import os
import sys

cwd = os.getcwd()
cam = cv2.VideoCapture(sys.argv[1], cv2.CAP_V4L2)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cv2.namedWindow("test", cv2.WINDOW_NORMAL)

img_directory = cwd +"/"+ sys.argv[1].split("/")[3]
img_counter = 0

if not os.path.exists(img_directory):
    os.makedirs(img_directory)

while True:
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow("test", frame)

    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "{}/{}.png".format(img_directory,img_counter)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1

cam.release()

cv2.destroyAllWindows()
