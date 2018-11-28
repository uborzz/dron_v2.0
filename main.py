# -*- coding: utf-8 -*-

import sys
import cv2
import numpy as np
import rady_locator
from datetime import datetime, timedelta
import config.config_devices as cfg

# Custom libs
import globales as gb
from rady_stream import Stream
import rady_functions as rfs
import dron
from rady_controller import Controller
from rady_configurator import Configurator
from video_writer import video_writer


####################################################################
###############  PREPARACIÓN e INICIALIZACION  #####################
####################################################################

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

# matriz y coefs. de distorsión de la lente
undistort, map1, map2, roi = rfs.get_undistort_map()

timestamp = "{:%Y_%m_%d_%H_%M}".format(datetime.now())
print("Timestamp:", timestamp)

gb.frame = None
panel = np.zeros((400,400,3), np.uint8)
gb.kalman_angle = True
gb.disable_all_kalmans = False
gb.solo_buscar_en_cercanias = True
gb.avanza, gb.retrocede = False, False

# params
distancia_camara_suelo = cfg.altura_camara
gb.refresca_params_flag = False


# Target (inicial) - Provisional
gb.xTarget = 320 # pixeles
gb.yTarget = 240 # pixeles
gb.zTarget = 20 # cm  # Pruebas corona hechas con Z 20
gb.angleTarget = 180 # grados


# prov.
gb.traza_aux = 0


recorder = rfs.Recorder()   # salva matriz de datos para analisis
recorder.configure(identificador=timestamp, fps=gb.fps)

configurator = Configurator()   # clase para configuracion con sliders
configurator.initialize(cfg.pid_config_inicial)

gb.info = False     # muestra/oculta info en consola

# Instancia el dron (selecciona puerto serial) y elige controller
midron = dron.create_dron(cfg.arduino_port, simulated=cfg.ignore_arduino)  # LABO
controller = Controller(info=False)
controller.initialize_general()
configurator.attach_controller(controller)
controller.control_simple_pid_init()

# instancia localizador del dron
locator = rady_locator.Localizador(distancia_camara_suelo, debug=False, info=gb.info, entorno=cfg.entorno)

video = video_writer()

# Callback clicks en ventana
def click_clases(event, x, y, flags, param):
    rfs.evalua_click(event, x, y, dron=midron, controller=controller, localizador=locator, frame=gb.frame)


def main():

    # mirror seleccionado?
    rotate = cfg.camera_rotation # realmente es ahora un ROTATE 180º
    # cam = Stream(src=0, resolution=(width, height), framerate=fps_camera).start()  # Tercera camara SRC = 2 - CASA = 0 - Unica
    cam = Stream(src=cfg.camera_src, resolution=(width, height), framerate=fps_camera).start()  # Segunda camara SRC = 1 (primera es la del portatil - 0)
    # cam = Stream(src="http://172.17.18.124:8080/?action=stream", resolution=(width, height), framerate=fps_camera).start()  # Lee stream de video por http


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


        # TODO AISLAR ESTO
        gb.angleTarget = cv2.getTrackbarPos("A Target", "target")
        gb.zTarget = cv2.getTrackbarPos("Z Target", "target")
        controller.control_pidlib_target(z = gb.zTarget)
        #############################

        # _, frame = cam.read() # Con videoCapture de openCv a pelo
        gb.frame, gb.frame_time = cam.read()  # Con mi STREAM de video

        # Espejo/rotate si/no?
        if rotate:
            gb.frame = cv2.flip(gb.frame, -1)  # 0 eje x, 1 eje y. -1 ambos (rota 180).

        # Camara calibrada? Obtenemos la transformacion
        if undistort:
            undistorted = cv2.remap(gb.frame, map1, map2, cv2.INTER_LINEAR)
            x, y, w, h = roi
            gb.frame = undistorted[y:y + h, x:x + w] # cropea el frame unudistorted

        # SCALE Imagen
        # frame = cv2.resize(frame, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_CUBIC)
        # cv2.imshow('image', frame)

        k = cv2.waitKey(1)
        if k != -1 and rfs.evalua_key(key_pressed=k, dron=midron, controller=controller, camera=cam, localizador=locator, frame=gb.frame):
            break

        # Localizador
        localizacion, estado_localizador = locator.posicion_dron(gb.frame)  # asignar a clase dron su localizador seria lo suyo.
        # Coger var que indike OK - NOK, o grado de OK del dron.

        # # valor auxiliar para relacionar log programa con imagen video.
        # print("Traza Aux:", str(gb.traza_aux))
        # gb.traza_aux += 1

        gb.x, gb.y, gb.z, gb.head = localizacion
        if datetime.now() - gb.timerStart <= timedelta(seconds=2):
            continue


        # Comandos de control:
        controller.run_meta_selector(windup=False)  # todo: dar una vuelta a esto.
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
        rfs.pinta_informacion_en_frame(gb.frame, midron, controller, t_frame=ms_frame)
        cv2.imshow('frame gordo', gb.frame)

        # rfs.pinta_informacion_en_panel_info(panel, midron, controller)
        # cv2.imshow('more info', panel)


    # SALVA VIDEO

        if video.is_enabled():
            video.write(gb.frame)

    cv2.destroyAllWindows()

    recorder.dump_to_file()

    if configurator.salvar_al_salir:
        configurator.save_config_file(gb.config_activa)

        # Provisional...
        locator.save_values_to_file()

    if video.is_initialized():
        video.release()
        print("If activated, video for this session can be found in...", video.get_filename())

    if not cfg.ignore_arduino:
        midron.panic()
        midron.close()

    print("See ya! =)")


if __name__ == '__main__':
    main()
