import cv2
from rady_stream import Stream
from scripts_pruebas.tests_rady_stream import Stream as Stream2
from datetime import datetime, timedelta
from time import sleep
from skimage.measure import compare_ssim
import imutils

# globales o similar
referencia = None
frame = None
umbral = 50


def set_referencia(frame):
    global referencia
    referencia = frame.copy()

def cambia_umbral(value):
    # callback del slider
    global umbral
    umbral = value

def magia_potagia(imageA, imageB):
    global umbral
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (abs(diff) * 255).astype("uint8")
    print("SSIM: {}".format(score))

    threshOTSU = cv2.threshold(diff, umbral, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    # otsu mola, automaticamente calcula el offset entre 2 los picos del histograma,
    # pasandose por los huevos nuestro umbral
    # https://docs.opencv.org/3.4.0/d7/d4d/tutorial_py_thresholding.html
    threshBININV = cv2.threshold(diff, umbral, 255, cv2.THRESH_BINARY_INV)[1]
    threshTRUNC = cv2.threshold(diff, umbral, 255, cv2.THRESH_TRUNC)[1]

    # otsu binary (invertido) tras desenfoke gausiano de la resta
    blur = cv2.GaussianBlur(diff, (5, 5), 0)
    threshGAUSS = cv2.threshold(blur, umbral, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    # otsu binary inv tras resta de los las imagenes con desen. gaussiano
    blurA = cv2.GaussianBlur(grayA, (5, 5), 0)
    blurB = cv2.GaussianBlur(grayB, (5, 5), 0)
    (scoreBLURR, diffBLURR) = compare_ssim(blurA, blurB, full=True)
    diffBLURR = (abs(diffBLURR) * 255).astype("uint8")
    threshGAUSSPREV = cv2.threshold(diffBLURR, umbral, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]

    # show the output images
    cv2.imshow("Diff", diff)
    cv2.imshow("DiffBLURR", diffBLURR)
    cv2.imshow("OTSU", threshOTSU)
    cv2.imshow("BININV", threshBININV)
    cv2.imshow("TRUNC", threshTRUNC)
    cv2.imshow("GAUSS", threshGAUSS)
    cv2.imshow("PREV", threshGAUSSPREV)


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

sleep(1)

cv2.namedWindow("controles")
cv2.createTrackbar("umbral", "controles", 40, 255, cambia_umbral)


while True:
    #
    toca_procesar = datetime.now() - timer_fps >= micros_para_procesar
    if not toca_procesar:
        continue



    timer_fps = datetime.now()
    frame = cam.read()
    # frame2 = cam2.read()
    if frame is not None:
        cv2.imshow("general", frame)
    if referencia is not None:
        cv2.imshow("referencia", referencia)

        magia_potagia(frame, referencia)

    # cv2.imshow("general2", frame2)
    k = cv2.waitKey(1)
    if k == ord("r"):
        set_referencia(frame)
    elif k == ord("q"):
        break

cv2.destroyAllWindows()
