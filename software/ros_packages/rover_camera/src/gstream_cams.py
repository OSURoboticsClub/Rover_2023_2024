"""
import cv2

gstreamer_str = (
    "v4l2src device=/dev/rover/camera_infrared ! videorate ! "
    "video/x-raw,framerate=25/1,width=640,height=480 ! nvvidconv ! "
    "video/x-raw,format=BGR ! appsink sync=false"
)

cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Unable to open GStreamer pipeline.")
else:
    print("GStreamer pipeline opened successfully.")

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        cv2.imshow("Input via Gstreamer", frame)
        print(frame)  # Fixed typo from "pritn" to "print"
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        print("Error: Unable to read frame.")
        break

cap.release()
cv2.destroyAllWindows()
"""
import cv2

gstreamer_red_str = (
    "v4l2src device=/dev/rover/camera_infrared ! videorate ! "
    "video/x-raw,framerate=25/1,width=640,height=480 ! "
    "videoconvert ! video/x-raw,format=BGR ! appsink sync=false"
)
gstreamer_nav_str = (
    "v4l2src device=/dev/rover/camera_main_navigation ! videorate ! "
    "video/x-raw,framerate=25/1,width=640,height=480 ! "
    "videoconvert ! video/x-raw,format=BGR ! appsink sync=false"
)
gstreamer_chas_str = (
    "v4l2src device=/dev/rover/camera_chassis ! videorate ! "
    "video/x-raw,framerate=25/1,width=640,height=480 ! "
    "videoconvert ! video/x-raw,format=BGR ! appsink sync=false"
)


cap_red = cv2.VideoCapture(gstreamer_red_str, cv2.CAP_GSTREAMER)
cap_nav = cv2.VideoCapture(gstreamer_nav_str, cv2.CAP_GSTREAMER)
cap_chas = cv2.VideoCapture(gstreamer_chas_str, cv2.CAP_GSTREAMER)
i = 0
while True:
    ret_red, frame_red = cap_red.read()
    ret_nav, frame_nav = cap_nav.read()
    ret_chas, frame_chas = cap_chas.read()

    print(i)
    i +=1
"""
    if ret_red:    
        cv2.imshow("Infrared", frame_red)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    if ret_nav:
        cv2.imshow("nav", frame_nav)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

#    if ret_chas:
#        cv2.imshow("chas", frame_chas)
    else:
        print("Error: Unable to read frame.")
        break
cap_nav.release()
cap_red.release()
cv2.destroyAllWindows()
"""
