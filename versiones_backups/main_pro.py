# -*- coding: utf-8 -*-

# pruebas en paralelo para desarrollo de trackeo con cámara no-cenital o composición de varias cámaras

from filevideostreamEnh import FileVideoStream
from imutils.video import FPS, VideoStream
import argparse
import imutils
import sys
import cv2
import numpy as np
import time
import dron
import localizador
from datetime import datetime, timedelta
import scipy.io
from scipy.signal import butter, lfilter, filtfilt
import json
import math
from rady_stream import Stream


VUELA = False
SALVA = False

# Version INFO
vpy = sys.version_info
print("Python Version: {}.{}.{}".format(vpy[0],vpy[1],vpy[2]))
print("OpenCV Version: " + cv2.__version__)
print("Numpy Version: " + np.__version__)

# Resolution
width = 640
height = 480
fps = 14  # Sampleo de imagenes. - Ajustado a ojímetro para el tiempo de procesamiento
fps_camera = 16 # a mano, fps de captura de la camara (va en otro hilo).
# fps_camera = int(fps + fps*0.15)  # framerate de captura camara (camara la llevo a otro hilo) - Un poco mas alto que el sampleo
# fps_envio  # A ojete tambien

try:
    raise("DESACTIVADO")
    # Con la correccion ralentizamos un 15-20% másaswa
    # Aprox. observado: 20 fps OK con corrección de la distorsiónq vs 24 fps OK sin corrección - en 640x480

    # load the camera calibration parameters
    calfile = np.load('calibration.npz')
    newcameramtx = calfile['newcameramtx']
    mtx = calfile['mtx']
    dist = calfile['dist']
    roi = calfile['roi']

    map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (width, height), cv2.CV_32FC1)
    undistort = True
    print("Camera calibration params loaded")

except:
    undistort = False
    print("WARNING: Going without camera calibration")

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

timestamp = "{:%Y_%m_%d_%H_%M}".format(datetime.now())
print("Timestamp:", timestamp)

frame = None
hsv = None
numero_click = 0

# butterworth
order = 2  # second order filter
fs = float(fps)  # sampling frequency is around 30 Hz
nyq = 0.5 * fs
lowcut = 1  # cutoff frequency at 2 Hz
low = lowcut / nyq
low_angle = 0.3 / nyq
b, a, *_ = butter(order, [low], btype='lowpass', output='ba') # tiro en _ para evitar intellisense highlight
ba, aa, *_ = butter(order, [low_angle], btype='lowpass', output='ba') # tiro en _ para evitar intellisense highlight
print("B - A:", b, "-", a)

# params
distancia_camara_suelo = 200
refresca_params_flag = False

# hover
xTarget = [320] # pixeles
yTarget = [240] # pixeles
zTarget = [20] # cm  # Pruebas corona hechas con Z 20
angleTarget = [210] # grados

# record everything
timeRecord = []
xRecord = []
xFilteredRecord = []
yRecord = []
yFilteredRecord = []
zRecord = []
zFilteredRecord = []
angleRecord = []
angleFilteredRecord = []
xErrorRecord = []
yErrorRecord = []
zErrorRecord = []
angleErrorRecord = []
aileronRecord = []
elevatorRecord = []
throttleRecord = []
rudderRecord = []

xError, yError, zError, angleError = 0, 0, 0, 0
xErrorI, yErrorI, zErrorI, angleErrorI = 0, 0, 0, 0
# xErrorD, yErrorD, zErrorD, angleErrorD 3= 0, 0, 0, 0
# xError_old, yError_old, zError_old, angleError_old = 0, 0, 0, 0

# Lee Config BIAS y PID
with open("config_E011.json", "r") as file:
    config_data = json.load(file)
    pid_data = config_data["pid"]
    bias_data = config_data["bias"]

# define middle PPM values that make the drone hover
throttle_middle = bias_data["throttle"]  # up (+) and down (-)  # 1660 con corona de 4 gramos
aileron_middle = bias_data["aileron"]  # left (-) and right (+)
elevator_middle = bias_data["elevator"]  # forward (+) and backward (-)
rudder_middle = bias_data["rudder"]  # yaw left (-) and yaw right (+)
correccion_gravedad = bias_data["correccion_gravedad"]

clamp_offset = 300

# marcial PID INICIALES - pixeles (x-y) - cm (z) - degree (head)
KPx, KPy, KPz, KPangle = pid_data["KPx"], pid_data["KPy"], pid_data["KPz"], pid_data["KPangle"]
KIx, KIy, KIz, KIangle = pid_data["KIx"], pid_data["KIy"], pid_data["KIz"], pid_data["KIangle"]
KDx, KDy, KDz, KDangle = pid_data["KDx"], pid_data["KDy"], pid_data["KDz"], pid_data["KDangle"]

# define engines off PPM value for throttle
throttle_off = 1000


# Instancia el dron - elige el "COM"
# if VUELA: midron = dron.Dron("COM6")
midron = dron.Dron("COM6")

info = True
locator = localizador.Localizador(distancia_camara_suelo, debug=False, info=info)


def windup():
    global xErrorI, yErrorI, zErrorI, angleErrorI
    xErrorI, yErrorI, zErrorI, angleErrorI = 0, 0, 0, 0

def windupXY():
    global xErrorI, yErrorI
    xErrorI, yErrorI = 0, 0

def click(event,x,y,flags,param):

    global frame
    global hsv
    global lower_blue
    global upper_blue

    # if event == cv2.EVENT_LBUTTONDOWN:
    #     pass

    if event == cv2.EVENT_LBUTTONUP:
        print("Color picker!")
        color = hsv[y,x]
        print(color)

        lower_blue = color.astype('int16') - 30
        upper_blue = color.astype('int16') + 30
        np.clip(lower_blue, 0, 255, out=lower_blue)
        np.clip(upper_blue, 0, 255, out=upper_blue)
        lower_blue[2] = 0
        upper_blue[2] = 255
        print("L" + str(lower_blue))
        print("U" + str(upper_blue))


        # cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
        # cv2.imshow("image", i


def colorea(color = "blanco"):
    if color == "verde":            res = (0, 255, 0)
    elif color == "cyan":           res = (255, 255, 0)
    elif color == "amarillo":       res = (0, 255, 255)
    elif color == "azul":           res = (255, 0, 0)
    elif color == "rojo":           res = (0, 0, 255)
    elif color == "negro":          res = (0,0,0)
    else:                           res = (255, 255, 255)  # blanco
    return res


# define a clamping function
def clamp(n, minimum, maximum):
    return max(min(maximum, n), minimum)


def click_clases(event, x, y, flags, param):

    global frame
    global hsv

    global numero_click
    global xTarget, yTarget, angleTarget

    # if event == cv2.EVENT_LBUTTONDOWN:
    #     pass

    if event == cv2.EVENT_LBUTTONUP:
        print("Color picker!")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color = hsv[y, x]
        print(color)
        print(color)
        print(color)
        print(color)

        # Nuevos targets

        # angleTarget = [np.degrees(np.arctan( (x - xTarget[0]) / (y - yTarget[0]) ) ) + 180] # grados
        xTarget = [x]  # pixeles
        yTarget = [y]  # pixeles
        windupXY()
        print(x, y)


        a = int(math.degrees(math.atan2(locator.coronaNaranja.x - x, locator.coronaNaranja.y - y))) + 180
        cv2.setTrackbarPos("A Target", "target", a)
        print(a)
        #
        # if numero_click == 0:
        #     locator.circuloVerde.calculate_hsv_range(color, 30)
        #     # locator.circuloVerde.set_range(lower, upper)
        # elif numero_click == 1:
        #     locator.circuloAmarillo.calculate_hsv_range(color, 30)
        #     # locator.circuloAmarillo.set_range(lower, upper)
        # elif numero_click == 2:
        #     locator.coronaNaranja.calculate_hsv_range(color, 20)
        #     # locator.coronaNaranja.set_range(lower, upper)
        #
        # numero_click += 1
        # if numero_click >= 3:
        #     numero_click = 0

def tracking():
    global VUELA, SALVA

    global frame
    global hsv

    global upper_blue
    global lower_blue

    global KPx, KPy, KPz, KPangle, KIx, KIy, KIz, KIangle, KDx, KDy, KDz, KDangle
    global refresca_params_flag
    global throttle_middle, rudder_middle, aileron_middle, elevator_middle, correccion_gravedad, clamp_offset

    global xError, yError, zError, angleError
    global xErrorI, yErrorI, zErrorI, angleErrorI


    ii = 0 # variable para moverse por puntos path

    # mirror seleccionado?
    mirror = True # realmente es ahora un ROTATE 180º

    ##### Con VIDEOSTREAM de IMUTILS -> Solo tiene sentido con camara raspi, es una mierda
    # cam = VideoStream(src=1, resolution=(width, height), framerate=fps).start()

    cam = Stream(src=1, resolution=(width, height), framerate=fps_camera).start()  # Segunda camara SRC = 1 (primera es la del portatil - 0)
    cam2 = Stream(src=2, resolution=(width, height), framerate=fps_camera).start()
    # cam = Stream(src=2, resolution=(width, height), framerate=fps_camera).start()  # Tercera camara SRC = 2

    ##### Con VIDEOCAPTURE A PELO.
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

    # # Instancia el dron
    # midron = dron.Dron("COM7")
    # locator = localizador.Localizador()

    image_number = 1

    # Lectura frames per second
    start = time.time()
    buffer = RingBuffer(10)
    avg_fps = 0.0
    counter = 0

    def refresca_params():
        global refresca_params_flag
        refresca_params_flag = False

        global KPx, KPy, KPz, KPangle, KIx, KIy, KIz, KIangle, KDx, KDy, KDz, KDangle
        global throttle_middle, rudder_middle, aileron_middle, elevator_middle, correccion_gravedad, clamp_offset

        KPz = cv2.getTrackbarPos("KPZ", "control")
        KIz = cv2.getTrackbarPos("KIZ", "control") / float(100)
        KDz = cv2.getTrackbarPos("KDZ", "control")
        throttle_middle = cv2.getTrackbarPos("BIASZ", "control")
        correccion_gravedad = cv2.getTrackbarPos("CG", "control")

        clamp_offset = cv2.getTrackbarPos("Clamp", "control")

        KPx = cv2.getTrackbarPos("KPX", "control") / float(100)
        KIx = cv2.getTrackbarPos("KIX", "control") / float(100)
        KDx = cv2.getTrackbarPos("KDX", "control")
        aileron_middle = cv2.getTrackbarPos("BIASAIL", "control")

        KPy = cv2.getTrackbarPos("KPY", "control") / float(100)
        KIy = cv2.getTrackbarPos("KIY", "control") / float(100)
        KDy = cv2.getTrackbarPos("KDY", "control")
        elevator_middle = cv2.getTrackbarPos("BIASELE", "control")

        KPangle = cv2.getTrackbarPos("KPA", "control")
        KIangle = cv2.getTrackbarPos("KIA", "control") / float(100)
        KDangle = cv2.getTrackbarPos("KDA", "control")
        rudder_middle = cv2.getTrackbarPos("BIASRUD", "control")



    def nothing(value):
        global refresca_params_flag
        refresca_params_flag = True

    def nothing_target():
        pass

    # clicks
    cv2.namedWindow('frame gordo')
    cv2.setMouseCallback('frame gordo', click_clases)


    # target
    cv2.namedWindow('target')
    cv2.resizeWindow('target', 300, 120)
    cv2.createTrackbar('Z Target', 'target', 20, 50, nothing_target)
    cv2.createTrackbar('A Target', 'target', 180, 360, nothing_target)


    # trackbars
    cv2.namedWindow('control')
    cv2.resizeWindow('control', 400, 750)
    cv2.createTrackbar('KPZ', 'control', KPz, 20, nothing)
    cv2.createTrackbar('KIZ', 'control', int(KIz*100), 100, nothing)
    cv2.createTrackbar('KDZ', 'control', KDz, 200, nothing)
    cv2.createTrackbar('BIASZ', 'control', throttle_middle, 2000, nothing)
    cv2.createTrackbar('CG', 'control', correccion_gravedad, 25, nothing)

    cv2.createTrackbar('Clamp', 'control', clamp_offset, 500, nothing)

    cv2.createTrackbar('KPX', 'control', int(KPx*100), 500, nothing)
    cv2.createTrackbar('KIX', 'control', int(KIx*100), 100, nothing)
    cv2.createTrackbar('KDX', 'control', KDx, 200, nothing)
    cv2.createTrackbar('BIASAIL', 'control', aileron_middle, 2000, nothing)

    cv2.createTrackbar('KPY', 'control', int(KPy*100), 500, nothing)
    cv2.createTrackbar('KIY', 'control', int(KIy*100), 100, nothing)
    cv2.createTrackbar('KDY', 'control', KDy, 200, nothing)
    cv2.createTrackbar('BIASELE', 'control', elevator_middle, 2000, nothing)

    cv2.createTrackbar('KPA', 'control', KPangle, 20, nothing)
    cv2.createTrackbar('KIA', 'control', int(KIangle*100), 100, nothing)
    cv2.createTrackbar('KDA', 'control', KDangle, 200, nothing)
    cv2.createTrackbar('BIASRUD', 'control', rudder_middle, 2000, nothing)

    # cv2.createTrackbar("kk", "oo", 0, 179, nothing)

    timerStart = datetime.now()
    timer_fps = datetime.now() # guarda tiempo de ultimo procesamiento.
    micros_para_procesar = timedelta(microseconds=int(1000000 / fps))

    starting = True

    while True:

        toca_procesar = datetime.now() - timer_fps >= micros_para_procesar
        if not toca_procesar:
            continue
        timer_fps = datetime.now()


        timer = cv2.getTickCount()
        t_start = datetime.now()

        if refresca_params_flag: refresca_params()
        angleTarget[0] = cv2.getTrackbarPos("A Target", "target")
        zTarget[0] = cv2.getTrackbarPos("Z Target", "target")

        if starting:
            print("Esperando inicializacion camara...")
            time.sleep(1)
            starting = False
        # _, frame = cam.read() # Con videoCapture a pelo
        frame = np.concatenate((cam.read(), cam2.read()), axis=1)

        # print(frame)

        # Espejo si/no?
        if mirror:
            frame = cv2.flip(frame, -1) # 0 eje x, 1 eje y. -1 ambos (rota 180).

        # Camara calibrada? Obtenemos la transformacion
        if undistort:
            # cropea el frame unudistorted
            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            x, y, w, h = roi
            frame = undistorted[y:y + h, x:x + w]

        # SCALE DOWN
        # frame = cv2.resize(frame, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_CUBIC)

        # cv2.imshow('image', frame)


        k = cv2.waitKey(1)
        if k == ord('q'):
            SALVA = True
            break
        elif k == 27:
            break
        elif k == ord('a'):
            midron.panic()
            windup() # chapuza windup
            VUELA = False
        elif k == ord('s'):
            totalenvios = midron.prueba_arduino_envios
            midron.panic()
            windup() # chapuza windup
            VUELA = False
            leidobuffer = midron.port.read(10000)
            leidolist = leidobuffer.splitlines()
            print(leidobuffer)
            print("Mandados: {} - Respuestas: {}".format(totalenvios, len(leidolist)))
        elif k == ord('r'):
            windup()
            midron.prueba_arduino_envios = 0
            # modo = "despega"  # Por aislar PID para cambiar sus parametros al vuelo.
            VUELA = True
        elif k == ord('c'):
            midron.calibrate2()
        elif k == ord('x'):
            midron.calibrate()
        elif k == ord('z'):
            # midron.send_command("1500,1700,1470,1500,0,0,0,1")
            midron.send_command("1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,2000")
            time.sleep(0.2)
            lectura = midron.port.read(2000)
            print("Lectura INIT enviado: ", lectura)
            midron.send_command("0,0,0,0,0,0,0,0,0,0,0,0")
            time.sleep(0.2)
            lectura = midron.port.read(1000)
            print("Saliendo INIT: ", lectura)
            # time.sleep(1)
        # elif k == ord('x'):
        #     midron.send_command("1500,1700,1470,1500,2000")
        #     time.sleep(0.2)
        #     midron.send_command("1500,1700,2000,1500")
        #     time.sleep(0.5)
        #     midron.send_command("1500,1700,1470,1500")
        #     # time.sleep(1)
        # elif k == ord('a'):
        #     midron.send_command("1500,1600,1470,1500")
        #     # time.sleep(1)

        # Aislar...
        x, y, z, head = locator.posicion_dron(frame)
        # Coger var que indice OK - NOK, o grado de OK del dorn.

        if datetime.now() - timerStart <= timedelta(seconds=2):
            continue

        ## CONTROL A PELO - Hover - (?) - Pruebas

        # if dron esta visto oK:
        xDrone, yDrone, zDrone, angleDrone = x, y, z, head

        # else frames without drone:
        #


        # record the position and orientation
        xRecord.append(xDrone)
        yRecord.append(yDrone)
        zRecord.append(zDrone)
        angleRecord.append(angleDrone)

        # filter y
        buff = 30
        if len(xRecord) >= buff:
            xFiltered = filtfilt(b, a, xRecord[-buff:])
            xDroneFiltered = xFiltered[-1]
            xFilteredRecord.append(xDroneFiltered)
            yFiltered = filtfilt(b, a, yRecord[-buff:])
            yDroneFiltered = yFiltered[-1]
            yFilteredRecord.append(yDroneFiltered)

            # filter z
            # zFiltered = lfilter(b, a, zRecord)
            zFiltered = filtfilt(b, a, zRecord[-buff:])
            zDroneFiltered = zFiltered[-1]
            zFilteredRecord.append(zDroneFiltered)

            # filter angle
            # angleFiltered = lfilter(b, a, angleRecord)
            angleFiltered = lfilter(ba, aa, angleRecord[-buff:])
            angleDroneFiltered = angleFiltered[-1]
            angleFilteredRecord.append(angleDroneFiltered)

        else:
            xFilteredRecord.append(xDrone)
            yFilteredRecord.append(yDrone)
            zFilteredRecord.append(zDrone)
            angleFilteredRecord.append(angleDrone)
            xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFiltered = xDrone, yDrone, zDrone, angleDrone
        # implement a PID controller

        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old = xError
        yError_old = yError
        zError_old = zError
        angleError_old = angleError

        # compute errors in position (cm) and angle (degrees) (P)
        # compute errors wrt filtered measurements
        if ii == len(xTarget):
            ii = 0

        # # PRUEBAS SIN FILTRO BUTTERWORTH (tenemos FILTRO DE KALMAN en la visión)
        # xError = xTarget[ii] - xDrone#Filtered
        # yError = yTarget[ii] - yDrone#Filtered
        # zError = zTarget[ii] - zDrone#Filtered
        # angleError = angleTarget[ii] - angleDrone#Filtered

        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        xError = xTarget[ii] - xDroneFiltered
        yError = yTarget[ii] - yDroneFiltered
        zError = zTarget[ii] - zDroneFiltered
        angleError = angleTarget[ii] - angleDroneFiltered

        # Trucar gravedad intento 1:
        if zError < 0 and correccion_gravedad > 0: zError /= correccion_gravedad
        # if zError < 0: print(zError)

        # Fix angulo, xq no habré hecho to en radianes? U-.-
        if angleError > 180: angleError -= 360
        elif angleError < -180: angleError += 360

        ii += 1

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        xErrorI += xError
        yErrorI += yError
        zErrorI += zError
        angleErrorI += angleError

        # compute derivative (variation) of errors (D)
        xErrorD = xError - xError_old
        yErrorD = yError - yError_old
        zErrorD = zError - zError_old
        angleErrorD = angleError - angleError_old

        # compute commands
        xCommand = KPx * xError + KIx * xErrorI + KDx * xErrorD
        yCommand = KPy * yError + KIy * yErrorI + KDy * yErrorD
        zCommand = KPz * zError + KIz * zErrorI + KDz * zErrorD
        angleCommand = KPangle * angleError + KIangle * angleErrorI + KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        # throttle command is zCommand
        # commands are relative to the middle PPM values
        throttleCommand = throttle_middle + zCommand

        # angleDrone to radians for projection

        angleDir = 360 - 180 + math.atan2(xCommand, yCommand) / math.pi*180
        # angleDir = 360 - 180 + math.atan2(xError, yError) / math.pi*180   <- Buena!
        angleMovRad = (angleDrone * np.pi / 180) - (angleDir * np.pi / 180)
        distMov = math.sqrt(xError**2 + yError**2)
        print("ANGULO RELATIVO: " + str(angleMovRad / math.pi*180))

        # project xCommand and yCommand on the axis of the drone
        # commands are relative to the middle PPM values
        elevatorCommand = elevator_middle - np.cos(angleMovRad) * distMov
        aileronCommand = aileron_middle - np.sin(angleMovRad) * distMov
        # elevatorCommand = elevator_middle - np.sin(angleMovRad) * xCommand + np.cos(angleMovRad) * yCommand
        # aileronCommand = aileron_middle - np.cos(angleMovRad) * xCommand - np.sin(angleMovRad) * yCommand

        # rudder command is angleCommand
        # commands are relative to the middle PPM values
        rudderCommand = rudder_middle - angleCommand

        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        # clamp_offset = 100
        throttleCommand = round(clamp(throttleCommand, throttle_middle-clamp_offset, throttle_middle+clamp_offset))
        throttleCommand = round(clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(clamp(aileronCommand, aileron_middle-clamp_offset, aileron_middle+clamp_offset))
        aileronCommand = round(clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(clamp(elevatorCommand, elevator_middle-clamp_offset, elevator_middle+clamp_offset))
        elevatorCommand = round(clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(clamp(rudderCommand, rudder_middle-clamp_offset, rudder_middle+clamp_offset))
        rudderCommand = round(clamp(rudderCommand, 1000, 2000))

        # create the command to send to Arduino
        command = "%i,%i,%i,%i" % (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)

        # print the projected commands
        if info: print("[COMMANDS]: T={:.0f} A={:.0f} E={:.0f} R={:.0f}".format(throttleCommand, aileronCommand, elevatorCommand, rudderCommand))

        # send to Arduino via serial port
        if VUELA: midron.send_command(command)
        # if VUELA: midron.set_command(command)
        # print("SE ENVIAN COMANDOS!!!")


        # record everything
        timerStop = datetime.now() - timerStart
        timeRecord.append(timerStop.total_seconds())

        xErrorRecord.append(xError)
        yErrorRecord.append(yError)
        zErrorRecord.append(zError)
        angleErrorRecord.append(angleError)

        elevatorRecord.append(elevatorCommand)
        aileronRecord.append(aileronCommand)
        throttleRecord.append(throttleCommand)
        rudderRecord.append(rudderCommand)


    #PINTA MIERDA EN PANTALLA

        # CURRENT INFO
        cv2.putText(frame, "X: " + str(x), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("amarillo"), 1)
        cv2.putText(frame, "Y: " + str(y), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("amarillo"), 1)
        cv2.putText(frame, "Z: " + str(int(z)), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("amarillo"), 1)
        cv2.putText(frame, "H: " + str(head), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("amarillo"), 1)
        fps_display = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        ms_frame = (datetime.now() - t_start).microseconds / 1000
        # cv2.putText(frame, "FPS: " + str(fps_display), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("verde"), 1)
        cv2.putText(frame, "T_FRAME: " + str(ms_frame), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("verde"), 1)

        # Target POINT
        cv2.putText(frame, "X: " + str(xTarget), (530, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("azul"), 1)
        cv2.putText(frame, "Y: " + str(yTarget), (530, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("azul"), 1)
        cv2.putText(frame, "Z: " + str(zTarget), (530, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("azul"), 1)
        cv2.putText(frame, "H: " + str(angleTarget), (530, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("azul"), 1)
        cv2.circle(frame, (xTarget[0], yTarget[0]), 3, colorea("azul"), -1)

        cv2.imshow('frame gordo', frame)


class RingBuffer():
    "A 1D ring buffer using numpy arrays"
    def __init__(self, length):
        self.length = length
        self.data = np.zeros(length, dtype='f')
        self.index = 0

    def extend(self, x):
        "adds array x to ring buffer"
        x_index = (self.index + np.arange(1)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

    def get(self):
        "Returns the first-in-first-out data in the ring buffer"
        idx = (self.index + np.arange(self.data.size)) % self.data.size
        return self.data[idx]

    def mean(self):
        return (self.data.mean())

def main():
    # show_webcam(640, 480, 30, mirror=True)
    # faster_webcam(1)
    tracking()

    cv2.destroyAllWindows()
    # if VUELA: midron.panic()
    # if VUELA: midron.close()

    midron.panic()
    midron.close()

    nombre_fichero = 'recordings/recording' + timestamp + '.mat'
    scipy.io.savemat(nombre_fichero,
                     mdict={'time': timeRecord, 'x': xRecord, 'xFiltered': xFilteredRecord, 'y': yRecord,
                            'yFiltered': yFilteredRecord, 'z': zRecord, 'zFiltered': zFilteredRecord,
                            'angle': angleRecord, 'angleFiltered': angleFilteredRecord, 'xError': xErrorRecord,
                            'yError': yErrorRecord, 'zError': zErrorRecord, 'angleError': angleErrorRecord,
                            'aileron': aileronRecord, 'elevator': elevatorRecord, 'throttle': throttleRecord,
                            'rudder': rudderRecord})
    diffs = np.ediff1d(timeRecord)
    tiempos = [sum(diffs[v * fps:(v * fps) + fps]) for v in range(int(diffs.shape[0] / fps))]
    print(tiempos)
    print(np.mean(tiempos))
    print("Fichero guardado con recordings: ", nombre_fichero)

    if SALVA:
        config_data = dict()
        pid_data = {
            "KPx" : KPx, "KPy": KPy, "KPz": KPz, "KPangle": KPangle,
            "KDx": KDx, "KDy": KDy, "KDz": KDz, "KDangle": KDangle,
            "KIx": KIx, "KIy": KIy, "KIz": KIz, "KIangle": KIangle,
        }
        bias_data = {
            "throttle": throttle_middle, "rudder": rudder_middle,
            "aileron": aileron_middle, "elevator": elevator_middle,
            "correccion_gravedad": correccion_gravedad
        }
        config_data["pid"] = pid_data
        config_data["bias"] = bias_data

        with open('config_E011.json', 'w') as file:
            json.dump(config_data, file, sort_keys=True, indent=4)

if __name__ == '__main__':
    main()