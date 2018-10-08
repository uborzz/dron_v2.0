# -*- coding: utf-8 -*-

import sys
import cv2
import numpy as np
import time
import localizador
from datetime import datetime, timedelta
import json
import math
import config_devices as cfg

# Custom libs
import globals as gb
from rady_stream import Stream
import rady_functions as rfs
import dron
from rady_controller import Controller
from rady_configurator import Configurator

# Version INFO
vpy = sys.version_info
print("Python Version: {}.{}.{}".format(vpy[0],vpy[1],vpy[2]))
print("OpenCV Version: " + cv2.__version__)
print("Numpy Version: " + np.__version__)


# Resolution
width, height = 640, 480
gb.fps = 20  # Sampleo de imagenes. - Ajustado a ojímetro para el tiempo de procesamiento
fps_camera = 25 # a mano, fps de captura de la camara (va en otro hilo).
# fps_camera = int(fps + fps*0.15)  # framerate de captura camara (camara la llevo a otro hilo) - Un poco mas alto que el sampleo


undistort, map1, map2, roi = rfs.get_undistort_map()

timestamp = "{:%Y_%m_%d_%H_%M}".format(datetime.now())
print("Timestamp:", timestamp)

frame = None
panel = np.zeros((400,400,3), np.uint8)
gb.kalman_angle = True
gb.disable_all_kalmans = False
gb.solo_buscar_en_cercanias = True

# params
distancia_camara_suelo = 200
gb.refresca_params_flag = False


# Target (inicial)
gb.xTarget = 320 # pixeles
gb.yTarget = 240 # pixeles
gb.zTarget = 20 # cm  # Pruebas corona hechas con Z 20
gb.angleTarget = 180 # grados


recorder = rfs.Recorder()
recorder.configure(identificador=timestamp, fps=gb.fps)

configurator = Configurator()


gb.info = False

# Instancia el dron - elige el "COM"
midron = dron.create_dron(cfg.arduino_port, simulated=cfg.ignore_arduino)  # LABO
controller = Controller(info=False)
controller.initialize_general()

locator = localizador.Localizador(distancia_camara_suelo, debug=False, info=gb.info, entorno=cfg.entorno)

#############################################################################
#  Funcionamiento deberia ser segun el modo, provisionalmente aqui
#  TODO Arreglar esta guarrada y aislar bien.
#############################################################################
def click_clases(event, x, y, flags, param):
    global frame

    # TODO Auto set range color para el locator (corona y ciruclo)
    if midron.mode == "CALIB_COLOR":
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
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            bgr = frame[y, x]
            color = hsv[y, x]
            print("HSV", color)
            print("HSV", color)
            print("BGR", bgr)
            print("BGR", bgr)

            # Nuevos targets

            # angleTarget = [np.degrees(np.arctan( (x - gb.xTarget[0]) / (y - yTarget[0]) ) ) + 180] # grados
            gb.xTarget = x  # pixeles
            gb.yTarget = y  # pixeles
            controller.windupXY()
            print("target point:")
            print(x, y)


            a = 180 + int(math.degrees(math.atan2(locator.coronaNaranja.x - x, locator.coronaNaranja.y - y)))
            if a >= 360: a-=360
            cv2.setTrackbarPos("A Target", "target", a)
            print("target angulo:")
            print(a)

            controller.control_simple_pid_init()
            print("PID de la lib simple_pid seteados valores target.")
#############################################################################

def main():
    global frame

    # mirror seleccionado?
    rotate = False # realmente es ahora un ROTATE 180º
    # cam = Stream(src=0, resolution=(width, height), framerate=fps_camera).start()  # Tercera camara SRC = 2 - CASA = 0 - Unica
    cam = Stream(src=cfg.camera_src, resolution=(width, height), framerate=fps_camera).start()  # Segunda camara SRC = 1 (primera es la del portatil - 0)
    # cam = Stream(src="http://172.17.18.124:8080/?action=stream", resolution=(width, height), framerate=fps_camera).start()  # Intento Http

    # # DESACTIVADO PROVISIONAL...
    # # Lectura frames per second
    # start = time.time()
    # buffer = rfs.RingBuffer(10)
    # avg_fps = 0.0
    # counter = 0

    # clicks
    cv2.namedWindow('frame gordo')
    cv2.moveWindow('frame gordo', 200, 100)
    cv2.setMouseCallback('frame gordo', click_clases)

    # cv2.namedWindow('more info')

    gb.timerStart = datetime.now()
    timer_fps = datetime.now()  # guarda tiempo de ultimo procesamiento.
    micros_para_procesar = timedelta(microseconds=int(1000000 / gb.fps))

    while True:

        toca_procesar = datetime.now() - timer_fps >= micros_para_procesar
        if not toca_procesar:
            continue
        timer_fps = datetime.now()

        timer_tick_count = cv2.getTickCount()
        t_loop_start = datetime.now()

        if gb.refresca_params_flag: configurator.panels.refresca_params()
        gb.angleTarget = cv2.getTrackbarPos("A Target", "target")
        gb.zTarget = cv2.getTrackbarPos("Z Target", "target")

        # _, frame = cam.read() # Con videoCapture a pelo
        frame = cam.read()  # Con STREAM

        # Espejo/rotate si/no?
        if rotate: frame = cv2.flip(frame, -1)  # 0 eje x, 1 eje y. -1 ambos (rota 180).

        # Camara calibrada? Obtenemos la transformacion
        if undistort:
            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            x, y, w, h = roi
            frame = undistorted[y:y + h, x:x + w] # cropea el frame unudistorted

        # SCALE Imagen
        # frame = cv2.resize(frame, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_CUBIC)
        # cv2.imshow('image', frame)

        k = cv2.waitKey(1)
        if rfs.evalua_key(key_pressed=k, dron=midron, controller=controller, camera=cam, localizador=locator, frame=frame):
            break

        # Localizador
        localizacion, estado_localizador = locator.posicion_dron(frame)  # asignar a clase dron su localizador seria lo suyo.
        # Coger var que indike OK - NOK, o grado de OK del dron.
        gb.x, gb.y, gb.z, gb.head = localizacion
        if datetime.now() - gb.timerStart <= timedelta(seconds=2):
            continue

        # Comandos de control:
        pack = controller.control()


        # Envío comandos a Arduino-Dron
        if pack: # and estado_localizador:
            (gb.throttle, gb.aileron, gb.elevator, gb.rudder) = pack
            if gb.info: print("[COMMANDS]: T={:.0f} A={:.0f} E={:.0f} R={:.0f}".format(gb.throttle, gb.aileron, gb.elevator, gb.rudder))
            command = "%i,%i,%i,%i" % (gb.throttle, gb.aileron, gb.elevator, gb.rudder)
            if midron.flag_vuelo: midron.send_command(command)
        else:
            if gb.info: print("no init control...")


    #PINTA MIERDA EN PANTALLA

        # fps_display = cv2.getTickFrequency() / (cv2.getTickCount() - timer_tick_count)
        ms_frame = (datetime.now() - t_loop_start).microseconds / 1000
        rfs.pinta_informacion_en_frame(frame, midron, controller, t_frame=ms_frame)
        cv2.imshow('frame gordo', frame)

        # rfs.pinta_informacion_en_panel_info(panel, midron, controller)
        # cv2.imshow('more info', panel)

    cv2.destroyAllWindows()

    midron.panic()
    midron.close()

    recorder.dump_to_file()

    if configurator.salvar_al_salir:
        configurator.save_config_file(gb.config_activa)


if __name__ == '__main__':
    main()
