import cv2
from rady_stream import Stream
from time import sleep
from datetime import datetime

vs = Stream(src=1, resolution=(640, 480), framerate=25).start()
frame = vs.read()
punto_click = None
foto = False

cv2.namedWindow("principal")


while True:
    # grab the frame from the threaded video +
    frame = vs.read()
    if punto_click:
        cv2.circle(frame, punto_click, 5, (255, 255, 255))

    cv2.imshow("principal", frame)
    k = cv2.waitKey(1)
    if k != -1:
        print("Code of key pressed:", k)
        if k == 27 or k == ord('q'):
            break





cv2.destroyAllWindows()