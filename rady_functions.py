# -*- coding: utf-8 -*-

import numpy as np
import cv2
import globals as gb
import scipy.io
import time
from rady_configurator import Configurator

configurator = Configurator()

def get_undistort_map():
    try:
        raise ("DESACTIVADO") # trick
        # Con la correccion ralentizamos un 15-20% másaswa
        # Aprox. observado: 20 fps OK con corrección de la distorsiónq vs 24 fps OK sin corrección - en 640x480

        # load the camera calibration parameters
        calfile = np.load('camera_calibration/calibration.npz')
        newcameramtx = calfile['newcameramtx']
        mtx = calfile['mtx']
        dist = calfile['dist']
        roi = calfile['roi']

        map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (width, height), cv2.CV_32FC1)
        undistort = True
        print("Camera calibration params loaded")

    except:
        map1, map2, roi = None, None, None
        undistort = False
        print("WARNING: Going without camera calibration")

    return undistort, map1, map2, roi


def clamp(n, minimum, maximum):
    return max(min(maximum, n), minimum)


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


def tupla_BGR(color = "blanco"):
    if color == "verde":            res = (0, 255, 0)
    elif color == "cyan":           res = (255, 255, 0)
    elif color == "amarillo":       res = (0, 255, 255)
    elif color == "azul":           res = (255, 0, 0)
    elif color == "rojo":           res = (0, 0, 255)
    elif color == "negro":          res = (0,0,0)
    else:                           res = (255, 255, 255)  # blanco
    return res


class Recorder(object):
    # Guarda info pasada de las posiciones, errores y demás.
    __instance = None

    def __new__(cls):
        if cls.__instance == None:
            cls.__instance = object.__new__(cls)
            cls.__instance.initialize()
        return cls.__instance

    def initialize(self):
        # record everything
        self.fps = None
        self.timeRecord = []
        self.xRecord = []
        self.xFilteredRecord = []
        self.yRecord = []
        self.yFilteredRecord = []
        self.zRecord = []
        self.zFilteredRecord = []
        self.angleRecord = []
        self.angleFilteredRecord = []
        self.angleMovidoRecord = []
        self.angleTuneadoRecord = []
        self.xErrorRecord = []
        self.yErrorRecord = []
        self.zErrorRecord = []
        self.angleErrorRecord = []
        self.aileronRecord = []
        self.elevatorRecord = []
        self.throttleRecord = []
        self.rudderRecord = []

    def configure(self, identificador, fps=None):
        self.fps = fps
        self.identificador = identificador

    def save_position(self, x, y, z, a):
        self.xRecord.append(x)
        self.yRecord.append(y)
        self.zRecord.append(z)
        self.angleRecord.append(a)

    def save_filtered_positions(self, x, y, z, a):
        self.xFilteredRecord.append(x)
        self.yFilteredRecord.append(y)
        self.zFilteredRecord.append(z)
        self.angleTuneadoRecord.append(a)

    def save_debug(self, angulo_movido, angulo_filtrado_raw):
        self.angleMovidoRecord.append(angulo_movido)
        self.angleFilteredRecord.append(angulo_filtrado_raw)

    def save_errors(self, x, y, z, a):
        self.xErrorRecord.append(x)
        self.yErrorRecord.append(y)
        self.zErrorRecord.append(z)
        self.angleErrorRecord.append(a)

    def save_commands(self, ele, ail, thr, rud):
        self.elevatorRecord.append(ele)
        self.aileronRecord.append(ail)
        self.throttleRecord.append(thr)
        self.rudderRecord.append(rud)

    def save_time(self, t):
        self.timeRecord.append(t)

    def dump_to_file(self):
        nombre_fichero = 'recordings/recording' + self.identificador + '.mat'
        scipy.io.savemat(nombre_fichero,
                     mdict={'time': self.timeRecord, 'x': self.xRecord, 'xFiltered': self.xFilteredRecord,
                            'y': self.yRecord, 'yFiltered': self.yFilteredRecord, 'z': self.zRecord,
                            'zFiltered': self.zFilteredRecord, 'angle': self.angleRecord,
                            'angleFiltered': self.angleFilteredRecord, 'angleMovido': self.angleMovidoRecord,
                            'angleTuneado': self.angleTuneadoRecord, 'xError': self.xErrorRecord,
                            'yError': self.yErrorRecord, 'zError': self.zErrorRecord,
                            'angleError': self.angleErrorRecord, 'aileron': self.aileronRecord,
                            'elevator': self.elevatorRecord, 'throttle': self.throttleRecord,
                            'rudder': self.rudderRecord})
        if self.fps:
            diffs = np.ediff1d(self.timeRecord)
            tiempos = [sum(diffs[v * self.fps:(v * self.fps) + self.fps]) for v in range(int(diffs.shape[0] / self.fps))]
            print(tiempos)
            print(np.mean(tiempos))
        print("Fichero guardado con recordings: ", nombre_fichero)


def pinta_informacion_en_frame(frame, fps=None, t_frame=None):
    # CURRENT INFO
    cv2.putText(frame, "X: " + str(gb.x), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    cv2.putText(frame, "Y: " + str(gb.y), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    cv2.putText(frame, "Z: " + str(int(gb.z)), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    cv2.putText(frame, "H: " + str(gb.head), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    # if fps: cv2.putText(frame, "FPS: " + str(fps_display), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("verde"), 1)
    if t_frame: cv2.putText(frame, "T_FRAME: " + str(t_frame), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("verde"), 1)

    # Target POINT
    cv2.putText(frame, "X: " + str(gb.xTarget), (530, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)
    cv2.putText(frame, "Y: " + str(gb.yTarget), (530, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)
    cv2.putText(frame, "Z: " + str(gb.zTarget), (530, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)
    cv2.putText(frame, "H: " + str(gb.angleTarget), (530, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)
    cv2.circle(frame, (gb.xTarget, gb.yTarget), 3, tupla_BGR("azul"), -1)

    cv2.putText(frame, "Config: " + gb.config_activa, (530, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)


def evalua_key(key_pressed, dron, controller):
    # Retorna True si el programa debe acabar
    k = key_pressed
    if k == ord('q'):
        configurator.salvar_al_salir = True
        return True
    elif k == 27:
        return True
    elif k == ord('a'):
        dron.panic()
        controller.windup()  # chapuza windup
        dron.flag_vuelo = True
    elif k == ord('s'):
        totalenvios = dron.prueba_arduino_envios
        dron.panic()
        controller.windup()  # chapuza windup
        dron.flag_vuelo = False
        leidobuffer = dron.port.read(10000)
        leidolist = leidobuffer.splitlines()
        print(leidobuffer)
        print("Mandados: {} - Respuestas: {}".format(totalenvios, len(leidolist)))
    elif k == ord('r'):
        controller.windup()
        dron.prueba_arduino_envios = 0
        # modo = "despega"  # Por aislar PID para cambiar sus parametros al vuelo.
        VUELA = True
    elif k == ord('c'):
        dron.calibrate2()
    elif k == ord('x'):
        dron.calibrate()
    elif k == ord('b'):
        dron.abajoizquierda()
    elif k == ord('m'):
        dron.abajoderecha()
    elif k == ord('n'):
        dron.neutro()
    elif k == ord('z'):
        # midron.send_command("1500,1700,1470,1500,0,0,0,1")
        dron.send_command("1000,1500,1500,1500,1000,1000,1000,1000,1000,1000,1000,2000")
        time.sleep(0.05)
        lectura = dron.port.read(1000)
        print("Lectura INIT enviado: ", lectura)
        # midron.send_command("1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000")
        dron.send_command("1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000")
        # midron.send_command("0,0,0,0,0,0,0,0,0,0,0,0")
        time.sleep(4)
        lectura = dron.port.read(1000)
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

        # Meter rotacion y salva/carga configs PID:
    elif k == ord('u'):
        configurator.save_to_config_activa()
    elif k == ord('i'):
        configurator.create_new_config_file()
    elif k == ord('o'):
        configurator.select_another_config_file()
    elif k == ord('p'):
        configurator.load_config_activa()