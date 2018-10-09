
### PRUEBAS: KeyPressed
# Pruebas construccion comandos para arduino presionando keys del teclado, para control manual.
##
# - rady

import serial
import time
import cv2

cam = cv2.VideoCapture(1)

throttle_middle = 1600  # up (+) and down (-)
aileron_middle = 1500  # left (-) and right (+)
elevator_middle = 1500  # forward (+) and backward (-)
rudder_middle = 1500  # yaw left (-) and yaw right (+)

# define engines off PPM value for throttle
throttle_off = 1000



thr, ail, ele, rud = 1600, aileron_middle, elevator_middle, rudder_middle

counter=0
paro = False

while True:

    counter+=1
    # print(counter)

    _, frame = cam.read()
    cv2.imshow('image', frame)

    k = cv2.waitKey(1)

    if k == ord("q"):
        thr, ail, ele, rud = 0, 0, 0, 0
        paro = True

    elif k == ord("w"):
        ele = 1300

    elif k == ord("a"):
        ail = 1300

    elif k == ord("s"):
        ele = 1700

    elif k == ord("d"):
        ail = 1700

    else:
        ail, ele = aileron_middle, elevator_middle

    if k == ord("o"):
        thr += 10

    elif k == ord("l"):
        thr -= 10

    else:
        if not paro:
            thr = 1600

    if k == 27:
        thr, ail, ele, rud = 0, 0, 0, 0
        command = "%i,%i,%i,%i" % (thr, ail, ele, rud)
        command = command + "\n"
        print("FINISHING")
        break

    command = "%i,%i,%i,%i" % (thr, ail, ele, rud)
    command = command + "\n"
    print(thr, ail, ele, rud)