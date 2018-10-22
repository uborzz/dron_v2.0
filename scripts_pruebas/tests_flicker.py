import cv2
from rady_stream import Stream
from scripts_pruebas.tests_rady_stream import Stream as Stream2
from datetime import datetime, timedelta
from time import sleep

# tests rapidos flicker tubos fluorescentes

width = 640
height = 480
fps_camera = 30

cam = Stream(src=1, resolution=(width, height), framerate=fps_camera).start()
# cam2 = Stream2(src=2, resolution=(width, height), framerate=fps_camera).start()

# mods...
fps=30
timer_fps = datetime.now()
micros_para_procesar = timedelta(microseconds=int(1000000 / fps))
cv2.namedWindow("general")
# cv2.namedWindow("general2")

sleep(2)

def callback_slider(value):
    cam.set_fps(value)

cv2.namedWindow('slider')
cv2.resizeWindow('slider', 300, 120)
cv2.createTrackbar('fps', 'slider', 10, 30, callback_slider)


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
    k = cv2.waitKey(1)

    if k == ord("q"):
        break
    elif k == ord("w"):
        cam.menu_config()

cv2.destroyAllWindows()