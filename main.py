# -*- coding: utf-8 -*-

import sys
import cv2
import numpy as np
import time
import localizador
from datetime import datetime, timedelta
import scipy.io
from scipy.signal import butter, lfilter, filtfilt
import json
import math
from rady_stream import Stream
import rady_functions
import globals as gb
import dron


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
fps = 20  # Sampleo de imagenes. - Ajustado a ojímetro para el tiempo de procesamiento
fps_camera = 25 # a mano, fps de captura de la camara (va en otro hilo).
# fps_camera = int(fps + fps*0.15)  # framerate de captura camara (camara la llevo a otro hilo) - Un poco mas alto que el sampleo
# fps_envio  # A ojete tambien

undistort, map1, map2 = rady_functions.get_undistort_map()

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

timestamp = "{:%Y_%m_%d_%H_%M}".format(datetime.now())
print("Timestamp:", timestamp)

frame = None
hsv = None
numero_click = 0

# butterworth X-Y
order = 2  # second order filter
fs = float(fps)  # sampling frequency is around 30 Hz
nyq = 0.5 * fs
lowcut = 1.1  # cutoff frequency at 1 Hz
low = lowcut / nyq
low_angle = 1.8 / nyq  # cutoff freq at 3 Hz
b, a, *_ = butter(order, [low], btype='lowpass', output='ba') # tiro en _ para evitar intellisense highlight
ba, aa, *_ = butter(order, [low_angle], btype='lowpass', output='ba') # tiro en _ para evitar intellisense highlight
print("B - A:", b, "-", a)


# butterworth Z


# params
distancia_camara_suelo = 200
gb.refresca_params_flag = False


# hover
gb.xTarget = 320 # pixeles
gb.yTarget = 240 # pixeles
gb.zTarget = 20 # cm  # Pruebas corona hechas con Z 20
gb.angleTarget = 210 # grados

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
angleMovidoRecord = []
angleTuneadoRecord = []
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
with open("config/config_E011.json", "r") as file:
    config_data = json.load(file)
    pid_data = config_data["pid"]
    bias_data = config_data["bias"]

# define middle PPM values that make the drone hover
gb.throttle_middle = bias_data["throttle"]  # up (+) and down (-)  # 1660 con corona de 4 gramos
gb.aileron_middle = bias_data["aileron"]  # left (-) and right (+)
gb.elevator_middle = bias_data["elevator"]  # forward (+) and backward (-)
gb.rudder_middle = bias_data["rudder"]  # yaw left (-) and yaw right (+)
gb.correccion_gravedad = bias_data["correccion_gravedad"]

gb.clamp_offset = 300

# marcial PID INICIALES - pixeles (x-y) - cm (z) - degree (head)
gb.KPx, gb.KPy, gb.KPz, gb.KPangle = pid_data["KPx"], pid_data["KPy"], pid_data["KPz"], pid_data["KPangle"]
gb.KIx, gb.KIy, gb.KIz, gb.KIangle = pid_data["KIx"], pid_data["KIy"], pid_data["KIz"], pid_data["KIangle"]
gb.KDx, gb.KDy, gb.KDz, gb.KDangle = pid_data["KDx"], pid_data["KDy"], pid_data["KDz"], pid_data["KDangle"]

# define engines off PPM value for throttle
throttle_off = 1000


# Instancia el dron - elige el "COM"
# if VUELA: midron = dron.Dron("COM6")
# midron = dron.Dron("COM6")
midron = dron.create_dron("COM6", simulated=False)

info = True
locator = localizador.Localizador(distancia_camara_suelo, debug=False, info=info)


def windup():
    global xErrorI, yErrorI, zErrorI, angleErrorI
    xErrorI, yErrorI, zErrorI, angleErrorI = 0, 0, 0, 0

def windupXY():
    global xErrorI, yErrorI
    xErrorI, yErrorI = 0, 0


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

        # angleTarget = [np.degrees(np.arctan( (x - gb.xTarget[0]) / (y - yTarget[0]) ) ) + 180] # grados
        gb.xTarget = x  # pixeles
        gb.yTarget = y  # pixeles
        windupXY()
        print("target point:")
        print(x, y)


        a = 180 + int(math.degrees(math.atan2(locator.coronaNaranja.x - x, locator.coronaNaranja.y - y)))
        if a >= 360: a-=360
        cv2.setTrackbarPos("A Target", "target", a)
        print("target angulo:")
        print(a)


def tracking():
    global VUELA, SALVA

    global frame
    global hsv

    global upper_blue
    global lower_blue

    global xError, yError, zError, angleError
    global xErrorI, yErrorI, zErrorI, angleErrorI


    # mirror seleccionado?
    rotate = True # realmente es ahora un ROTATE 180º

    cam = Stream(src=1, resolution=(width, height), framerate=fps_camera).start()  # Segunda camara SRC = 1 (primera es la del portatil - 0)
    # cam = Stream(src=2, resolution=(width, height), framerate=fps_camera).start()  # Tercera camara SRC = 2

    image_number = 1

    # Lectura frames per second
    start = time.time()
    buffer = rady_functions.RingBuffer(10)
    avg_fps = 0.0
    counter = 0


    # clicks
    cv2.namedWindow('frame gordo')
    cv2.setMouseCallback('frame gordo', click_clases)


    panels = rady_functions.ConfigPanels()


    # cv2.createTrackbar("kk", "oo", 0, 179, nothing)

    timerStart = datetime.now()
    timer_fps = datetime.now() # guarda tiempo de ultimo procesamiento.
    micros_para_procesar = timedelta(microseconds=int(1000000 / fps))

    # provisional:
    anguloPrevio = None

    while True:

        toca_procesar = datetime.now() - timer_fps >= micros_para_procesar
        if not toca_procesar:
            continue
        timer_fps = datetime.now()


        timer = cv2.getTickCount()
        t_start = datetime.now()

        if gb.refresca_params_flag: panels.refresca_params()
        gb.angleTarget = cv2.getTrackbarPos("A Target", "target")
        gb.zTarget = cv2.getTrackbarPos("Z Target", "target")

        # _, frame = cam.read() # Con videoCapture a pelo
        frame = cam.read() # Con VIDEOSTREAM
        # print(frame)

        # Espejo/rotate si/no?
        if rotate:
            frame = cv2.flip(frame, -1) # 0 eje x, 1 eje y. -1 ambos (rota 180).

        # Camara calibrada? Obtenemos la transformacion
        if undistort:
            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            x, y, w, h = roi
            frame = undistorted[y:y + h, x:x + w] # cropea el frame unudistorted

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
        elif k == ord('b'):
            midron.abajoizquierda()
        elif k == ord('m'):
            midron.abajoderecha()
        elif k == ord('n'):
            midron.neutro()
        elif k == ord('z'):
            # midron.send_command("1500,1700,1470,1500,0,0,0,1")
            midron.send_command("1000,1500,1500,1500,1000,1000,1000,1000,1000,1000,1000,2000")
            time.sleep(0.05)
            lectura = midron.port.read(1000)
            print("Lectura INIT enviado: ", lectura)
            # midron.send_command("1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000")
            midron.send_command("1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000")
            # midron.send_command("0,0,0,0,0,0,0,0,0,0,0,0")
            time.sleep(4)
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
        print(gb.KPz)

        # record the position and orientation
        xRecord.append(xDrone)
        yRecord.append(yDrone)
        zRecord.append(zDrone)
        angleRecord.append(angleDrone)

        # filter y
        buff = fps
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

            # Tuneo medida leida angulo
            if anguloPrevio:  # Previo y filtrado son el mismo valor. anguloPrevio y angleDroneFiltered
                while (angleDrone - anguloPrevio) > 180:
                    angleDrone -= 360
                while (angleDrone - anguloPrevio) <= -180:
                    angleDrone += 360
            angleMovidoRecord.append(angleDrone)

            # filter angle
            # angleFiltered = lfilter(b, a, angleRecord)
            angleFiltered = lfilter(ba, aa, angleMovidoRecord[-10:])    # a manota
            angleDroneFiltered = angleFiltered[-1]
            anguloPrevio = int(angleDroneFiltered)
            angleFilteredRecord.append(angleDroneFiltered)

            angleDroneFilteredTuneado = int(anguloPrevio)
            while angleDroneFilteredTuneado >= 360:
                angleDroneFilteredTuneado -= 360
            while angleDroneFilteredTuneado < 0:
                angleDroneFilteredTuneado += 360

            angleTuneadoRecord.append(angleDroneFilteredTuneado)

        else:
            angleMovidoRecord.append(angleDrone)
            angleTuneadoRecord.append(angleDrone)
            xFilteredRecord.append(xDrone)
            yFilteredRecord.append(yDrone)
            zFilteredRecord.append(zDrone)
            angleFilteredRecord.append(angleDrone)
            xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado = xDrone, yDrone, zDrone, angleDrone
        # implement a PID controller

        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old = xError
        yError_old = yError
        zError_old = zError
        angleError_old = angleError


        if info: print("[LEIDO FILTRADO]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado))
        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        xError = gb.xTarget - xDroneFiltered
        yError = gb.yTarget - yDroneFiltered
        zError = gb.zTarget - zDroneFiltered
        angleError = gb.angleTarget - angleDroneFilteredTuneado   # Pruebas angulos!!

        # Trucar gravedad intento 1:
        if zError < 0 and gb.correccion_gravedad > 0: zError /= gb.correccion_gravedad
        # if zError < 0: print(zError)

        # Fix angulo, xq no habré hecho to en radianes? U-.-
        if angleError > 180: angleError -= 360
        elif angleError <= -180: angleError += 360

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
        xCommand = gb.KPx * xError + gb.KIx * xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * yError + gb.KIy * yErrorI + gb.KDy * yErrorD
        zCommand = gb.KPz * zError + gb.KIz * zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * angleError + gb.KIangle * angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        # throttle command is zCommand
        # commands are relative to the middle PPM values
        throttleCommand = gb.throttle_middle + zCommand

        # angleDrone to radians for projection

# -> Provisional
        # ###############
        angleDir = 360 - 180 + math.atan2(xCommand, yCommand) / math.pi*180
        # angleDir = 360 - 180 + math.atan2(xError, yError) / math.pi*180   <- Buena!
        angleMovRad = (angleDroneFilteredTuneado * np.pi / 180) - (angleDir * np.pi / 180)     # <- FILTRADO O LEIDO?? TUN TUN
        distMov = math.sqrt(xError**2 + yError**2)
        # ###############


        # print("ANGULO RELATIVO: " + str(angleMovRad / math.pi*180))

        # project xCommand and yCommand on the axis of the drone
        # commands are relative to the middle PPM values

# -> Provisional
        # ###############
        elevatorCommand = gb.elevator_middle - np.cos(angleMovRad) * distMov
        aileronCommand = gb.aileron_middle - np.sin(angleMovRad) * distMov
        # ###############
        # elevatorCommand = elevator_middle - yCommand
        # aileronCommand = aileron_middle + xCommand

        # elevatorCommand = elevator_middle - np.sin(angleMovRad) * xCommand + np.cos(angleMovRad) * yCommand
        # aileronCommand = aileron_middle - np.cos(angleMovRad) * xCommand - np.sin(angleMovRad) * yCommand

        # rudder command is angleCommand
        # commands are relative to the middle PPM values
        rudderCommand = gb.rudder_middle - angleCommand

        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        # clamp_offset = 100
        throttleCommand = round(clamp(throttleCommand, gb.throttle_middle-gb.clamp_offset, gb.throttle_middle+gb.clamp_offset))
        throttleCommand = round(clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(clamp(aileronCommand, gb.aileron_middle-gb.clamp_offset, gb.aileron_middle+gb.clamp_offset))
        aileronCommand = round(clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(clamp(elevatorCommand, gb.elevator_middle-gb.clamp_offset, gb.elevator_middle+gb.clamp_offset))
        elevatorCommand = round(clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(clamp(rudderCommand, gb.rudder_middle-gb.clamp_offset, gb.rudder_middle+gb.clamp_offset))
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
        cv2.putText(frame, "X: " + str(gb.xTarget), (530, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("azul"), 1)
        cv2.putText(frame, "Y: " + str(gb.yTarget), (530, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("azul"), 1)
        cv2.putText(frame, "Z: " + str(gb.zTarget), (530, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("azul"), 1)
        cv2.putText(frame, "H: " + str(gb.angleTarget), (530, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colorea("azul"), 1)
        cv2.circle(frame, (gb.xTarget, gb.yTarget), 3, colorea("azul"), -1)

        cv2.imshow('frame gordo', frame)



def main():

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
                            'angle': angleRecord, 'angleFiltered': angleFilteredRecord,
                            'angleMovido': angleMovidoRecord, 'angleTuneado': angleTuneadoRecord,
                            'xError': xErrorRecord,
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
            "KPx" : gb.KPx, "KPy": gb.KPy, "KPz": gb.KPz, "KPangle": gb.KPangle,
            "KDx": gb.KDx, "KDy": gb.KDy, "KDz": gb.KDz, "KDangle": gb.KDangle,
            "KIx": gb.KIx, "KIy": gb.KIy, "KIz": gb.KIz, "KIangle": gb.KIangle,
        }
        bias_data = {
            "throttle": gb.throttle_middle, "rudder": gb.rudder_middle,
            "aileron": gb.aileron_middle, "elevator": gb.elevator_middle,
            "correccion_gravedad": gb.correccion_gravedad
        }
        config_data["pid"] = pid_data
        config_data["bias"] = bias_data

        with open('config_E011.json', 'w') as file:
            json.dump(config_data, file, sort_keys=True, indent=4)

if __name__ == '__main__':
    main()