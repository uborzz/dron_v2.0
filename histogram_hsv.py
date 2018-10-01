import cv2
import os
import matplotlib.pyplot as plt

file_list = os.listdir("captures")
fichero_activo = file_list[0]
img = cv2.imread("captures/" + fichero_activo, cv2.IMREAD_COLOR)

def click_clases(event, x, y, flags, param):
    global img
    # if event == cv2.EVENT_LBUTTONDOWN:
    #     pass
    if event == cv2.EVENT_LBUTTONUP:
        print("Color picker!")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        bgr = img[y, x]
        color = hsv[y, x]
        print("HSV", color)
        print("BGR", bgr)

        print("target point:")
        print(x, y)

def selecciona_siguiente(fichero_activo):
    index = file_list.index(fichero_activo) + 1
    if index >= len(file_list):
        index = 0
    fichero_activo =  file_list[index]
    print("Fichero seleccionado:", fichero_activo)
    return fichero_activo

cambio = True
cv2.namedWindow('imagen')
cv2.setMouseCallback('imagen', click_clases)

while True:

    cv2.imshow("imagen", img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)

    chans = cv2.split(hsv)
    print(len(chans))

    print(chans[0])

    hist = cv2.calcHist(chans[0], [0], None, [181], [0, 181])

    if cambio:
        plt.plot(hist, color="b")
        plt.xlim([0, 181])
        plt.show()
        cambio = False
    #
    # # hist = cv2.calcHist(chans[1], [0], None, [256], [0, 256])
    # # plt.plot(hist, color="b")
    # # plt.xlim([0, 256])
    # # plt.show()
    # #
    # # hist = cv2.calcHist(chans[2], [0], None, [256], [0, 256])
    # # plt.plot(hist, color="g")
    # # plt.xlim([0, 256])
    # # plt.show()
    #
    # cv2.imshow("imagen", img)
    # cv2.waitKey(1)
    #
    k = cv2.waitKey(1)
    if k == ord("q"):
        cambio = True
        fichero_activo = selecciona_siguiente(fichero_activo)
        img = cv2.imread("captures/" + fichero_activo, cv2.IMREAD_COLOR)


