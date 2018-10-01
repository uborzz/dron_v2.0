import cv2
from rady_stream import Stream
from scripts_pruebas.tests_rady_stream import Stream as Stream2
from datetime import datetime, timedelta
from time import sleep

width = 640
height = 480
fps_camera = 30

cam = Stream(src=1, resolution=(width, height), framerate=fps_camera).start()
# cam2 = Stream2(src=2, resolution=(width, height), framerate=fps_camera).start()

# mods...
fps=20
timer_fps = datetime.now()
micros_para_procesar = timedelta(microseconds=int(1000000 / fps))
cv2.namedWindow("general")
# cv2.namedWindow("general2")

sleep(2)

while True:
    #
    toca_procesar = datetime.now() - timer_fps >= micros_para_procesar
    if not toca_procesar:
        continue
    timer_fps = datetime.now()
    frame = cam.read()
    # frame2 = cam2.read()
    cv2.imshow("general", frame)
    # cv2.imshow("general2", frame2)
    cv2.waitKey(1)

# cam = cv2.VideoCapture(1)
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
# cam.set(cv2.CAP_PROP_FPS, fps)
# cam = cv2.VideoCapture(1)
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
# cam.set(cv2.CAP_PROP_FPS, fps)
# cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
# cam.set(cv2.CAP_PROP_RECTIFICATION, 0)
# cam.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 0)
# cam.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 0)