# -*- coding: utf-8 -*-

### TOOL: CAMERA CONFIGURATOR
# configura por teclas CONTRASTE, BRILLO y SATURACION, si lo soporta la camara/driver.
# ** REQUIERE video input desde cámara por la clase Stream de rady_stream.
#
# Como usar:
# teclas U I O : Suben C B S, respectivamente.
# teclas J K L : Bajan C B S, respectivamente.
#
# Info:
# Drivers ubuntu - windows tienen diferente escala.
# probado para C270 de Logitech.
#
# - rady

import cv2
from rady_stream import Stream

vs = Stream(src=1, resolution=(640, 480), framerate=25).start()
frame = vs.read()
punto_click = None
foto = False

def click_func(event, x, y, flags, param):
    global frame, punto_click
    # Clicamos 1 vez
    if event == cv2.EVENT_LBUTTONUP:
        punto_click = (x, y)
        print("CLICK CAPTURADO!")

        color = frame[y, x, :]
        print("BGR:", str(color))
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_color = frameHSV[y, x, :]
        print("HSV:", hsv_color)

cv2.namedWindow("principal")
cv2.setMouseCallback("principal", click_func)


while True:
    # grab the frame from the threaded video +
    frame = vs.read()
    if punto_click:
        cv2.circle(frame, punto_click, 5, (255, 255, 255))

    cv2.imshow("principal", frame)
    k = cv2.waitKey(1)

    if k == ord('q'):
        break
    elif k == ord('o'):
        vs.sube_saturacion()
    elif k == ord('l'):
        vs.baja_saturacion()
    elif k == ord('i'):
        vs.sube_brillo()
    elif k == ord('k'):
        vs.baja_brillo()
    elif k == ord('u'):
        vs.sube_contraste()
    elif k == ord('j'):
        vs.baja_contraste()


cv2.destroyAllWindows()