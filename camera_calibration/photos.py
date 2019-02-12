# -*- coding: utf-8 -*-

### TOOL: Camara Fotos
# Captura imagenes con los clicks del raton. Guardadas en carpeta captures
#
# - rady

import cv2
from rady_stream import Stream
from datetime import datetime, timedelta


vs = Stream(src=1).start()
frame = vs.read()
foto = False
t_start = datetime.now()


def click_func(event, x, y, flags, param):
    global frame
    global foto

    if event == cv2.EVENT_RBUTTONUP or event == cv2.EVENT_LBUTTONUP:
        print("FRAME CAPTURADO!")
        foto = True


def captura_foto(frame):
    global foto

    timestamp = "{:%Y%m%d%H%M%S}".format(datetime.now())
    fichero = "./captures/img_" + timestamp + '.jpg'
    cv2.imwrite(fichero, frame)
    foto = False


cv2.namedWindow("principal")
cv2.setMouseCallback("principal", click_func)

while True:
    # Main loop

    frame, _ = vs.read()

    # inicializando
    if datetime.now() - t_start <= timedelta(seconds=1):
        continue

    if foto:
        captura_foto(frame)

    cv2.imshow("principal", frame)

    k = cv2.waitKeyEx(1)
    if k == ord('q') or k == 27:
        break

print("See ya!")
cv2.destroyAllWindows()
