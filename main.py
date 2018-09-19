# -*- coding: utf-8 -*-

import sys
import cv2
import numpy as np
import time
import localizador
from datetime import datetime, timedelta
import json
import math
from rady_stream import Stream
import rady_functions as rfs
import globals as gb
import dron
from controller import Controller


VUELA = False
SALVA = False

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

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

timestamp = "{:%Y_%m_%d_%H_%M}".format(datetime.now())
print("Timestamp:", timestamp)

frame = None
hsv = None
numero_click = 0



                    # # butterworth X-Y
                    # order = 2  # second order filter
                    # fs = float(fps)  # sampling frequency is around 30 Hz
                    # nyq = 0.5 * fs
                    # lowcut = 1.1  # cutoff frequency at 1 Hz
                    # low = lowcut / nyq
                    # low_angle = 1.8 / nyq  # cutoff freq at 3 Hz
                    # b, a, *_ = butter(order, [low], btype='lowpass', output='ba') # tiro en _ para evitar intellisense highlight
                    # ba, aa, *_ = butter(order, [low_angle], btype='lowpass', output='ba') # tiro en _ para evitar intellisense highlight
                    # print("B - A:", b, "-", a)


# butterworth Z


# params
distancia_camara_suelo = 200
gb.refresca_params_flag = False


# hover
gb.xTarget = 320 # pixeles
gb.yTarget = 240 # pixeles
gb.zTarget = 20 # cm  # Pruebas corona hechas con Z 20
gb.angleTarget = 210 # grados


recorder = rfs.Recorder()
recorder.configure(identificador=timestamp, fps=gb.fps)

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
controller = Controller()
controller.initialize_general()

info = True
locator = localizador.Localizador(distancia_camara_suelo, debug=False, info=info)


def windup():
    global xErrorI, yErrorI, zErrorI, angleErrorI
    xErrorI, yErrorI, zErrorI, angleErrorI = 0, 0, 0, 0

def windupXY():
    global xErrorI, yErrorI
    xErrorI, yErrorI = 0, 0


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


    # # DESACTIVADO PROVISIONAL...
    # # Lectura frames per second
    # start = time.time()
    # buffer = rfs.RingBuffer(10)
    # avg_fps = 0.0
    # counter = 0


    # clicks
    cv2.namedWindow('frame gordo')
    cv2.setMouseCallback('frame gordo', click_clases)


    panels = rfs.ConfigPanels()


    # cv2.createTrackbar("kk", "oo", 0, 179, nothing)

    gb.timerStart = datetime.now()
    timer_fps = datetime.now() # guarda tiempo de ultimo procesamiento.
    micros_para_procesar = timedelta(microseconds=int(1000000 / gb.fps))


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
        frame = cam.read() # Con STREAM

        # Espejo/rotate si/no?
        if rotate: frame = cv2.flip(frame, -1) # 0 eje x, 1 eje y. -1 ambos (rota 180).

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
        gb.x, gb.y, gb.z, gb.head = locator.posicion_dron(frame)  # asignar a clase dron su localizador seria lo suyo.
        # Coger var que indike OK - NOK, o grado de OK del dron.

        if datetime.now() - gb.timerStart <= timedelta(seconds=2):
            continue


        # if dron está visto OK:
        throttle, aileron, elevator, rudder = controller.control()

                #
                #         # if dron esta visto oK:
                #         xDrone, yDrone, zDrone, angleDrone = x, y, z, head
                #
                #
                #         # ##########################
                #         # Filtros y PID control
                #
                #         # record the position and orientation
                #         recorder.save_position(xDrone, yDrone, zDrone, angleDrone)
                #
                #         # filter y
                #         buff = fps
                #         if len(recorder.xRecord) >= buff:
                #             xFiltered = filtfilt(b, a, recorder.xRecord[-buff:])
                #             xDroneFiltered = xFiltered[-1]
                #                     # xFilteredRecord.append(xDroneFiltered)
                #             yFiltered = filtfilt(b, a, recorder.yRecord[-buff:])
                #             yDroneFiltered = yFiltered[-1]
                #                     # yFilteredRecord.append(yDroneFiltered)
                #
                #             # filter z
                #             # zFiltered = lfilter(b, a, zRecord)
                #             zFiltered = filtfilt(b, a, recorder.zRecord[-buff:])
                #             zDroneFiltered = zFiltered[-1]
                #                     # zFilteredRecord.append(zDroneFiltered)
                #
                #             # Tuneo medida leida angulo
                #             if anguloPrevio:  # Previo y filtrado son el mismo valor. anguloPrevio y angleDroneFiltered
                #                 while (angleDrone - anguloPrevio) > 180:
                #                     angleDrone -= 360
                #                 while (angleDrone - anguloPrevio) <= -180:
                #                     angleDrone += 360
                #
                #                     # angleMovidoRecord.append(angleDrone)
                #
                #             # filter angle
                #             # angleFiltered = lfilter(b, a, angleRecord)
                #             angleFiltered = lfilter(ba, aa, recorder.angleMovidoRecord[-10:])    # a manota el 10
                #             angleDroneFiltered = angleFiltered[-1]
                #             anguloPrevio = int(angleDroneFiltered)
                #                     # angleFilteredRecord.append(angleDroneFiltered)
                #
                #             # El filtrado tuneado es el correcto, en rango 0-360 tras aplicar filtro
                #             angleDroneFilteredTuneado = int(anguloPrevio)
                #             while angleDroneFilteredTuneado >= 360:
                #                 angleDroneFilteredTuneado -= 360
                #             while angleDroneFilteredTuneado < 0:
                #                 angleDroneFilteredTuneado += 360
                #
                #             recorder.save_debug(angulo_movido=angleDrone, angulo_filtrado_raw=angleDroneFiltered)
                #             recorder.save_filtered_positions(xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado)
                #
                #         else:
                #             recorder.save_debug(angulo_movido=angleDrone, angulo_filtrado_raw=angleDrone)
                #             recorder.save_filtered_positions(xDrone, yDrone, zDrone, angleDrone)
                #             xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado = xDrone, yDrone, zDrone, angleDrone
                #         # implement a PID controller
                #
                #         # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
                #         xError_old, yError_old, zError_old, angleError_old = xError, yError, zError, angleError
                #
                #
                #         if info: print("[LEIDO FILTRADO]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado))
                #         # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
                #         xError = gb.xTarget - xDroneFiltered
                #         yError = gb.yTarget - yDroneFiltered
                #         zError = gb.zTarget - zDroneFiltered
                #         angleError = gb.angleTarget - angleDroneFilteredTuneado   # Pruebas angulos!!
                #
                #         # Trucar gravedad intento 1:
                #         if zError < 0 and gb.correccion_gravedad > 0: zError /= gb.correccion_gravedad
                #         # if zError < 0: print(zError)
                #
                #         # Fix angulo, xq no habré hecho to en radianes? U-.-
                #         if angleError > 180: angleError -= 360
                #         elif angleError <= -180: angleError += 360
                #
                #         # compute integral (sum) of errors (I)
                #         # should design an anti windup for z
                #         xErrorI += xError
                #         yErrorI += yError
                #         zErrorI += zError
                #         angleErrorI += angleError
                #
                #         # compute derivative (variation) of errors (D)
                #         xErrorD = xError - xError_old
                #         yErrorD = yError - yError_old
                #         zErrorD = zError - zError_old
                #         angleErrorD = angleError - angleError_old
                #
                #         # compute commands
                #         xCommand = gb.KPx * xError + gb.KIx * xErrorI + gb.KDx * xErrorD
                #         yCommand = gb.KPy * yError + gb.KIy * yErrorI + gb.KDy * yErrorD
                #         zCommand = gb.KPz * zError + gb.KIz * zErrorI + gb.KDz * zErrorD
                #         angleCommand = gb.KPangle * angleError + gb.KIangle * angleErrorI + gb.KDangle * angleErrorD
                #
                #         # print the X, Y, Z, angle commands
                #         # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
                #         # print("X:", str(xCommand))
                #
                #         throttleCommand = gb.throttle_middle + zCommand
                #
                #         # angleDrone to radians for projection
                #
                # # -> Provisional
                #         # ###############
                #         angleDir = 360 - 180 + math.atan2(xCommand, yCommand) / math.pi*180
                #         # angleDir = 360 - 180 + math.atan2(xError, yError) / math.pi*180   <- Buena!
                #         angleMovRad = (angleDroneFilteredTuneado * np.pi / 180) - (angleDir * np.pi / 180)     # <- FILTRADO O LEIDO?? TUN TUN
                #         distMov = math.sqrt(xError**2 + yError**2)
                #         # ###############
                #
                #
                #         # print("ANGULO RELATIVO: " + str(angleMovRad / math.pi*180))
                #
                #         # project xCommand and yCommand on the axis of the drone
                #         # commands are relative to the middle PPM values
                #
                # # -> Provisional
                #         # ###############
                #         elevatorCommand = gb.elevator_middle - np.cos(angleMovRad) * distMov
                #         aileronCommand = gb.aileron_middle - np.sin(angleMovRad) * distMov
                #         # ###############
                #         # elevatorCommand = elevator_middle - yCommand
                #         # aileronCommand = aileron_middle + xCommand
                #
                #         # elevatorCommand = elevator_middle - np.sin(angleMovRad) * xCommand + np.cos(angleMovRad) * yCommand
                #         # aileronCommand = aileron_middle - np.cos(angleMovRad) * xCommand - np.sin(angleMovRad) * yCommand
                #
                #         # rudder command is angleCommand
                #         # commands are relative to the middle PPM values
                #         rudderCommand = gb.rudder_middle - angleCommand
                #
                #         # round and clamp the commands to [1000, 2000] us (limits for PPM values)
                #         # clamp_offset = 100
                #         throttleCommand = round(clamp(throttleCommand, gb.throttle_middle-gb.clamp_offset, gb.throttle_middle+gb.clamp_offset))
                #         throttleCommand = round(clamp(throttleCommand, 1000, 2000))
                #         aileronCommand = round(clamp(aileronCommand, gb.aileron_middle-gb.clamp_offset, gb.aileron_middle+gb.clamp_offset))
                #         aileronCommand = round(clamp(aileronCommand, 1000, 2000))
                #         elevatorCommand = round(clamp(elevatorCommand, gb.elevator_middle-gb.clamp_offset, gb.elevator_middle+gb.clamp_offset))
                #         elevatorCommand = round(clamp(elevatorCommand, 1000, 2000))
                #         rudderCommand = round(clamp(rudderCommand, gb.rudder_middle-gb.clamp_offset, gb.rudder_middle+gb.clamp_offset))
                #         rudderCommand = round(clamp(rudderCommand, 1000, 2000))



        # create the command to send to Arduino
        command = "%i,%i,%i,%i" % (throttle, aileron, elevator, rudder)

        # print the projected commands
        if info: print("[COMMANDS]: T={:.0f} A={:.0f} E={:.0f} R={:.0f}".format(throttle, aileron, elevator, rudder))

        # send to Arduino via serial port
        if VUELA: midron.send_command(command)
        # if VUELA: midron.set_command(command)
        # print("SE ENVIAN COMANDOS!!!")


    #PINTA MIERDA EN PANTALLA

        # fps_display = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        ms_frame = (datetime.now() - t_start).microseconds / 1000
        rfs.pinta_informacion_en_frame(frame, t_frame=ms_frame)
        cv2.imshow('frame gordo', frame)



def main():

    tracking()

    cv2.destroyAllWindows()
    # if VUELA: midron.panic()
    # if VUELA: midron.close()

    midron.panic()
    midron.close()

                    # nombre_fichero = 'recordings/recording' + timestamp + '.mat'
                    # scipy.io.savemat(nombre_fichero,
                    #                  mdict={'time': timeRecord, 'x': xRecord, 'xFiltered': xFilteredRecord, 'y': yRecord,
                    #                         'yFiltered': yFilteredRecord, 'z': zRecord, 'zFiltered': zFilteredRecord,
                    #                         'angle': angleRecord, 'angleFiltered': angleFilteredRecord,
                    #                         'angleMovido': angleMovidoRecord, 'angleTuneado': angleTuneadoRecord,
                    #                         'xError': xErrorRecord,
                    #                         'yError': yErrorRecord, 'zError': zErrorRecord, 'angleError': angleErrorRecord,
                    #                         'aileron': aileronRecord, 'elevator': elevatorRecord, 'throttle': throttleRecord,
                    #                         'rudder': rudderRecord})
                    # diffs = np.ediff1d(timeRecord)
                    # tiempos = [sum(diffs[v * fps:(v * fps) + fps]) for v in range(int(diffs.shape[0] / fps))]
                    # print(tiempos)
                    # print(np.mean(tiempos))
                    # print("Fichero guardado con recordings: ", nombre_fichero)

    recorder.dump_to_file()

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