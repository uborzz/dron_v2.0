# -*- coding: utf-8 -*-

### TOOL: PRINT KEY PRESSED
# Canta codigo ASCII de la key apretada
#
# Como usar:
# Apretar las teclas teniendo activa la ventana principal del streaming
#
# Info:
# Ojo! Diferencias windows-ubuntu trato teclas shift, ctrl y alt.
#
# - rady

import cv2
from rady_stream import Stream

vs = Stream(src=1, resolution=(640, 480), framerate=25).start()
frame = vs.read()
punto_click = None
foto = False

cv2.namedWindow("principal")

while True:
    # grab the frame from the threaded video +
    frame, _= vs.read()
    if punto_click:
        cv2.circle(frame, punto_click, 5, (255, 255, 255))

    cv2.imshow("principal", frame)
    k = cv2.waitKey(1)
    j = cv2.waitKeyEx(1)
    if k != -1:
        print("waitkey captured:", k)
        if k == 27 or k == ord('q'):
            break
    if j != -1:
        print("waitkeyEx code:", j)




cv2.destroyAllWindows()