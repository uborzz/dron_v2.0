# -*- coding: utf-8 -*-

### TOOL: TAKE_COLOR y FOTOS
# Printea el color del pixel seleccionado y captura fotos
#
# Como usar:
# Click izquierdo: Lee color pixel clickado
# CLick derecho: Guarda frame en disco como imagen jpg
#
# - rady

import cv2
from imutils.video import VideoStream
from time import sleep
from datetime import datetime

vs = VideoStream(src=1).start()
frame = vs.read()
punto_click = None
foto = False

def click_func(event, x, y, flags, param):
    global frame
    global punto_click
    global foto

    # Clicamos 1 vez
    if event == cv2.EVENT_LBUTTONUP:
        punto_click = (x, y)
        print("CLICK CAPTURADO!")

        color = frame[y, x, :]
        print("BGR:", str(color))
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_color = frameHSV[y, x, :]
        print("HSV:", hsv_color)

    if event == cv2.EVENT_RBUTTONUP:
        print("FRAME CAPTURADO!")
        foto = True

cv2.namedWindow("principal")
cv2.setMouseCallback("principal", click_func)

while True:
    # grab the frame from the threaded video +
    frame = vs.read()
    if punto_click:
        cv2.circle(frame, punto_click, 5, (255, 255, 255))
    if foto:
        foto = False
        timestamp = "{:%Y_%m_%d_%H_%M}".format(datetime.now())
        fichero = "fotos/foto_" + timestamp
        cv2.imwrite(fichero, frame)
    cv2.imshow("principal", frame)
    cv2.waitKey(1)
