# -*- coding: utf-8 -*-

### FUNCTIONS
# Funciones auxiliares y clases provisionales, antes de pasarlas a un módulo aparte.
#
# Lo más importante...
# . get_undistort_map toma un frame y devuelve otro corregido con los parametros de calibracion de la camara
# . Recorder se encarga de ir almacenando valores previos en arrays y salvar a fichero para analisis
#       en realidad tambien se reutiliza para acceder a historia para filtros y demás.
# Tambien hay funciones para pintar info sobre el frame o sobre paneles.
# . evalua_key es un mega if-elif que se encarga de resolver tareas asignadas keys del teclado
#
# - rady

import numpy as np
import cv2
import globales as gb
import scipy.io
import math
import time
from rady_configurator import Configurator
from datetime import datetime, timedelta

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

        self.xErrorIRecord = []
        self.yErrorIRecord = []
        self.zErrorIRecord = []
        self.angleErrorIRecord = []
        self.xErrorDRecord = []
        self.yErrorDRecord = []
        self.zErrorDRecord = []
        self.angleErrorDRecord = []

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

    def save_errors(self, x, y, z, a, modo="p"):
        if modo=="p":
            self.xErrorRecord.append(x)
            self.yErrorRecord.append(y)
            self.zErrorRecord.append(z)
            self.angleErrorRecord.append(a)
        elif modo=="i":
            self.xErrorIRecord.append(x)
            self.yErrorIRecord.append(y)
            self.zErrorIRecord.append(z)
            self.angleErrorIRecord.append(a)
        elif modo=="d":
            self.xErrorDRecord.append(x)
            self.yErrorDRecord.append(y)
            self.zErrorDRecord.append(z)
            self.angleErrorDRecord.append(a)

    def save_commands(self, ele, ail, thr, rud):
        self.elevatorRecord.append(ele)
        self.aileronRecord.append(ail)
        self.throttleRecord.append(thr)
        self.rudderRecord.append(rud)

    def save_time(self, t):
        self.timeRecord.append(t)

    def dump_to_file(self):
        nombre_fichero = 'recordings/recording' + self.identificador + '.mat'
        try:
            scipy.io.savemat(nombre_fichero,
                         mdict={'time': self.timeRecord, 'x': self.xRecord, 'xFiltered': self.xFilteredRecord,
                                'y': self.yRecord, 'yFiltered': self.yFilteredRecord, 'z': self.zRecord,
                                'zFiltered': self.zFilteredRecord, 'angle': self.angleRecord,
                                'angleFiltered': self.angleFilteredRecord, 'angleMovido': self.angleMovidoRecord,
                                'angleTuneado': self.angleTuneadoRecord, 'xError': self.xErrorRecord,
                                'yError': self.yErrorRecord, 'zError': self.zErrorRecord,
                                'angleError': self.angleErrorRecord, 'aileron': self.aileronRecord,
                                'elevator': self.elevatorRecord, 'throttle': self.throttleRecord,
                                'rudder': self.rudderRecord,
                                'xErrorI': self.xErrorIRecord, 'yErrorI': self.yErrorIRecord, 'zErrorI': self.zErrorIRecord,
                                'angleErrorI': self.angleErrorIRecord,
                                'xErrorD': self.xErrorDRecord, 'yErrorD': self.yErrorDRecord, 'zErrorD': self.zErrorDRecord,
                                'angleErrorD': self.angleErrorDRecord
                                })
            print("Fichero guardado con recordings: ", nombre_fichero)
        except Exception as e:
            print("dump data to file failed with exception:", str(e))

        if self.fps and len(self.timeRecord):
            print("tiempos...")
            diffs = np.ediff1d(self.timeRecord)
            tiempos = [sum(diffs[v * self.fps:(v * self.fps) + self.fps]) for v in range(int(diffs.shape[0] / self.fps))]
            print(tiempos)
            print(np.mean(tiempos))
        else:
            print("No hay datos almacenados en el tiempo...")



def pinta_informacion_en_frame(frame, dron, controller, fps=None, t_frame=None):
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

    cv2.putText(frame, "Config: " + gb.config_activa, (10, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    cv2.putText(frame, "Control: " + controller.mode, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"),1)
    cv2.putText(frame, "Dron: " + dron.mode, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"),1)

    cv2.putText(frame, "kalmans: " + str(not gb.disable_all_kalmans), (440, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("verde"),1)
    cv2.putText(frame, "cercanias: " + str(gb.solo_buscar_en_cercanias), (440, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("verde"),1)


def pinta_informacion_en_panel_info(panel, dron, controller, fps=None, t_frame=None):
    # CURRENT INFO
    cv2.putText(panel, "X: " + str(gb.x), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    cv2.putText(panel, "Y: " + str(gb.y), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    cv2.putText(panel, "Z: " + str(int(gb.z)), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    cv2.putText(panel, "H: " + str(gb.head), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    # if fps: cv2.putText(frame, "FPS: " + str(fps_display), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("verde"), 1)

    # Target POINT
    cv2.putText(panel, "X: " + str(gb.xTarget), (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)
    cv2.putText(panel, "Y: " + str(gb.yTarget), (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)
    cv2.putText(panel, "Z: " + str(gb.zTarget), (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)
    cv2.putText(panel, "H: " + str(gb.angleTarget), (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("azul"), 1)
    cv2.putText(panel, "Config: " + gb.config_activa, (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"), 1)
    cv2.putText(panel, "Control: " + controller.mode, (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"),1)
    cv2.putText(panel, "Dron: " + dron.mode, (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tupla_BGR("amarillo"),1)


def evalua_key(key_pressed, dron, controller, camera, localizador, frame=None):
    # Retorna True si el programa debe acabaraaaaaaaaaaaaa

    """

        Q        |   W        |  E     |      R    |       T      |     Y
    save config  | camara     |        |   Run     | Run bias     |  photo to
        & quit   |   config   |        |           |    calib     |    captures
    |____________|____________|________|___________|______________|_____________
        A        |   S        |
      Parada     | Paro       |
        Soft     |   Gordo    |                                     [[Mandos]]
    |____________|____________|____________|            ____________|____________|____________|
        Z        |   X        |   C        |           |   B        |   N        |   M        |
        BIND     | Calibra    | Calibra    |           | Mandos     | Mandos     | Mandos     |
     No Bwhoop   |IMU Bwhoop  |IMU Eachine |           | (a-i) / /  | (aba) | |  |  (a-d)\ \  |


                                            |____________|
                                            |   !        |
                                            | on-off     |
                                            |  IMSHOWS   |
                 [[MODOS]]                      |____________|____
                                                |   +        |
                                                | on-off     |
                                                |CONSOLE INFO|
                     ____________|______________|____________|
                    |   ñ        |   ´        |   ç        |
                    | on-off     |            | on-off     |
                    |  Cercania  |            |  Kalman L  |
            ________|____________|____________|____________|
            |   ,        |   .        |   -        |
            |            |     Modo   |   Modo     |
            |            |   Dron     |    Control |


    Modo no "CALIB_COLOR" (Normal):
                ___________________________________________________
                U      |        I      |       O      |         P
           save config |  save config  |    select    |     activate
             actual    |  in new file  |  next config |  selected config


    Modo "CALIB_COLOR" - Modifica params del driver camara:
                    _______________________________________
                        U      |    I      |     O
                   contraste+  | brillo+   | saturacion+
                   ____________|___________|______________
                        J      |    K      |     L
                   contraste-  | brillo-   | saturacion-
                   ____________|___________|_______________



    """

    k = key_pressed
    if k == ord('q'):
        configurator.salvar_al_salir = True
        return True
    elif k == 27:
        return True
    elif k == ord('w'):
        camera.menu_config()
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

    elif k == ord('.'):
        dron.change_mode()
    elif k == ord('-'):
        controller.change_mode()
    elif k == ord('+'):
        gb.info = not gb.info

    elif k == ord('¡'):
        localizador.toggle_debug_images()

        # provisional
    elif k == ord('ç'):
        gb.disable_all_kalmans = not gb.disable_all_kalmans
    elif k == ord('ñ'):
        gb.solo_buscar_en_cercanias = not gb.solo_buscar_en_cercanias


    # extras
    elif k==ord('1'):   # MODO HACER VOLTERETA
        dron.prepara_modo(timedelta(seconds=0.5), gb.aileron)
        dron.set_mode("FLIP")

    elif k==ord('2'):   # MODO SALIR Y ENTRAR DE CÁMARA     # TODO afinar.
        dron.prepara_modo(timedelta(seconds=1.8), gb.aileron)
        dron.set_mode("EXPLORE")

    elif k==ord('3'):   # MODO ATERRIZAJE
        dron.prepara_modo(timedelta(seconds=4), gb.throttle)
        dron.set_mode("LAND")



    # PROVISIONAL - Funciones teclas si Modo del dron "CALIB_COLOR".
    elif dron.mode == "CALIB_COLOR":
        ## PROVISIONAL - TODO Pasar a sliders
        if k == ord('u'):
            camera.sube_contraste()
        elif k == ord('j'):
            camera.baja_contraste()
        elif k == ord('i'):
            camera.sube_brillo()
        elif k == ord('k'):
            camera.baja_brillo()
        elif k == ord('o'):
            camera.sube_saturacion()
        elif k == ord('l'):
            camera.baja_saturacion()

        # No parece que hagan nada con nuestro driver/camara ###############
        elif k == ord('t'):
            camera.sube_gamma()
        elif k == ord('g'):
            camera.baja_gamma()
        elif k == ord('y'):
            camera.sube_hue()
        elif k == ord('h'):
            camera.baja_hue()
        elif k == ord('e'):
            camera.sube_sharpness()
        elif k == ord('d'):
            camera.baja_sharpness()
        ####################################################################

        elif k == ord('r'):
            camera.sube_exposicion()
        elif k == ord('f'):
            camera.baja_exposicion()

        elif k == ord('s'):
            camera.read_camera_params()


    # Selector config file para el PID. - Provisional funciona solo si el modo del dron no es CALIB_COLOR
    # Meter rotacion y salva/carga configs PID:
    else:
        if k == ord('u'):   # SALVA CONFIG EN CONFIG ACTUAL
            configurator.save_to_config_activa()
        elif k == ord('i'): # SALVA CONFIG EN FICHERO NUEVO
            configurator.create_new_config_file()
        elif k == ord('o'): # SELECCIONA CONFIG SIGUIENTE
            configurator.select_another_config_file()
        elif k == ord('p'): # CARGA FICHERO CONFIG SELECCIONADO
            configurator.load_config_activa()


        elif k == ord('y'):  # foto
            img_name = "ss_{:%m%d%H%M%S}.jpg".format(datetime.now())
            print("Imagen guardada:", img_name)
            cv2.imwrite("captures/" + img_name, frame)

        elif k == ord('a'):
            dron.panic()
            controller.windup()  # chapuza windup
            dron.set_mode("NO_INIT")
            dron.flag_vuelo = False
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
            controller.control_simple_pid_init()
            dron.set_mode("DESPEGUE")
            # modo = "despega"  # Por aislar PID para cambiar sus parametros al vuelo.
            dron.flag_vuelo = True
        elif k == ord('f'):
            dron.prueba_arduino_envios = 0
            controller.control_pidlib_init()
            controller.set_mode("PIDLIB")
            configurator.load_config_file("config_pidlib.json")
            dron.set_mode("DESPEGUE")
            dron.flag_vuelo = True
        elif k == ord('t'):
            controller.windup()
            dron.prueba_arduino_envios = 0
            controller.t_inicio_maniobra = datetime.now()
            dron.set_mode("DESPEGUE")
            controller.set_mode("CALIB_BIAS")
            # modo = "despega"  # Por aislar PID para cambiar sus parametros al vuelo.
            dron.flag_vuelo = True


def evalua_click(event, x, y, dron, controller, localizador, frame):
    # TODO Auto set range color para el locator (corona y ciruclo)
    if dron.mode == "CALIB_COLOR":
        if event == cv2.EVENT_LBUTTONUP:
            print("Color picker!")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            bgr = frame[y, x]
            color = hsv[y, x]
            print("HSV", color)
            print("HSV", color)
            print("BGR", bgr)
            print("BGR", bgr)

        if event == cv2.EVENT_RBUTTONUP:
            print("Color picker!")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            bgr = frame[y, x]
            color = hsv[y, x]
            print("HSV", color)
            print("HSV", color)
            print("BGR", bgr)
            print("BGR", bgr)

    else:
        if event == cv2.EVENT_LBUTTONUP:
            print("Color picker!")

            # Nuevos targets

            # angleTarget = [np.degrees(np.arctan( (x - gb.xTarget[0]) / (y - yTarget[0]) ) ) + 180] # grados
            gb.xTarget = x  # pixeles
            gb.yTarget = y  # pixeles
            # controller.windupXY()     # 181025 - no resetear
            print("target point:")
            print(x, y)

            a = 180 + int(math.degrees(math.atan2(localizador.coronaNaranja.x - x, localizador.coronaNaranja.y - y)))
            if a >= 360: a -= 360
            cv2.setTrackbarPos("A Target", "target", a)
            print("target angulo:")
            print(a)


            # controller.control_simple_pid_init()
            # print("PID de la lib simple_pid seteados valores target.")

            controller.control_pidlib_target(x=x, y=y)
