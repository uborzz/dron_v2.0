# -*- coding: utf-8 -*-

### CONTROLLER
# Implementa las lógicas de control del dron, aisladas en diferentes funciones alternativas
# Mediante ifs y según el modo de control que indique el programa principal o el usuario s
# se llamará a un bloque (método) u otro.
#
# Dentro de cada bloque de control se lee la posición del dron (globales) y se estiman
# los siguientes comandos a enviara al dron para corregir su posición, normalmente usando PIDs
#
# - rady


import rady_functions as rfs
import dron
from scipy.signal import butter, lfilter, filtfilt
import globales as gb
import numpy as np
import math
from datetime import datetime, timedelta
from rady_configurator import Configurator
from simple_pid import PID

"""
https://github.com/m-lundberg/simple-pid
"""

recorder = rfs.Recorder()
configurator = Configurator()

class Controller():
    def __init__(self, info=False):
        self.print_info = info
        self.mode = "BASICO"
        self.modes_available = ["BASICO", "BASICO_LF", "CALIB_BIAS", "HOVER_BIAS", "BASICO_PID_LIB", "FILTROS_TESTS"]

    def initialize_general(self):
        # butterworth X-Y
        order = 2  # second order filter
        fs = float(gb.fps)  # sampling frequency is around 30 Hz
        nyq = 0.5 * fs
        lowcut = 1.1  # cutoff frequency at 1 Hz
        low = lowcut / nyq
        low_angle = 1.8 / nyq  # cutoff freq at 3 Hz
        self.b, self.a, *_ = butter(order, [low], btype='lowpass', output='ba')  # tiro en _ para evitar intellisense highlight
        self.ba, self.aa, *_ = butter(order, [low_angle], btype='lowpass', output='ba')
        print("B - A:", self.b, "-", self.a)


        # Pruebas 1 suavizado.
        tests_fs = float(gb.fps)
        tests_nyq = 0.5 * tests_fs
        tests_lowcut = 5  # Corte a 5Hz
        tests_low = tests_lowcut / tests_nyq
        print(tests_low, tests_nyq)
        self.bbb, self.aaa, *_ = butter(order, [tests_low], btype='lowpass', output='ba')

        # provisional:
        self.anguloPrev = None
        self.xError, self.yError, self.zError, self.angleError = 0, 0, 0, 0
        self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI = 0, 0, 0, 0

        self.t_inicio_maniobra = datetime.now()

        self.dron = dron.take_dron()

        self.forcing_down = False
        self.consecutive_frames_out_of_limits = 0

        self.t_frame_previo = datetime.now().timestamp()

        self.autoadjust = datetime.now()
        self.necesario_windup = False
        self.veces_autoajuste = 0
        self.max_veces_autoajuste = 2


        # MUY PROVISIONAL, PRUEBAS
        self.ignore_derivative_error = False
        self.provisional_planner = 0
        self.provisional_planner_ts = datetime.now()

        #... en la misma linea, prov. pruebas rapida para prueba funcionamiento.
        self.flag_esperando = False


    def change_mode(self):
        index = self.modes_available.index(self.mode) + 1
        if index >= len(self.modes_available):
            index = 0
        self.mode = self.modes_available[index]
        print("Modo controller elegido:", self.mode)

    def set_mode(self, mode):
        print("Activando controlador:", mode)
        self.mode = mode

    def windup(self):
        self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI = 0, 0, 0, 0

    def windupXY(self):
        self.xErrorI, self.yErrorI = 0, 0

    def windupZ(self):
        self.zErrorI = 0

    # Playground pruebas diferentes modos
    def control(self):
        # print("Entrando control")
        if self.dron.mode == "DESPEGUE":
            if self.mode == "BASICO":
                return self.control_basico()
                # return self.control_basico_piddoble()
                # return self.control_basico_factorsolointegral()
                # return self.control_basico()
            elif self.mode == "NOVIEMBRE":
                return self.control_noviembre()
            elif self.mode == "BASICO_DOBLE":
                return self.control_basico_piddoble()
            elif self.mode == "HOVER_BIAS":
                return self.control_hover_bias()
            elif self.mode == "BASICO_LF":
                return self.control_basico_lfilter()
            elif self.mode == "CALIB_BIAS":
                return self.control_calib_bias()
            elif self.mode == "BASICO_PID_LIB":
                return self.control_basico_pidzsimple()
                # return self.control_simple_pid()
            elif self.mode == "PIDLIB":
                return self.control_pidlib()
            elif self.mode == "FILTROS_TESTS":
                return self.control_basico_pruebas_filters()
            # return self.control_con_filtros()
        elif self.dron.mode == "HOLD":
            if self.mode == "BASICO":
                return self.control_basico_piddoble()
                # return self.control_basico_piddoble()
                # return self.control_basico_factorsolointegral()
                # return self.control_basico()
            elif self.mode == "NOVIEMBRE":
                return self.control_noviembre()
            elif self.mode == "BASICO_G_FACTOR":
                return self.control_basico()
            elif self.mode == "BASICO_PID_LIB":
                return self.control_basico_pidzsimple()
                # return self.control_simple_pid()
            elif self.mode == "PIDLIB":
                return self.control_pidlib()
            elif self.mode == "BASICO_LIMIT_Z":
                return self.control_land()
            elif self.mode == "BASICO_CLAMP_THROTTLE":
                return self.control_clamp_throttle()
            elif self.mode == "BASICO_DISABLE_GFACTOR":
                return self.control_disable_factor()
            # return self.control_con_filtros()
        elif self.dron.mode == "FLIP":
            return self.control_flip()
        elif self.dron.mode == "LAND":
            return self.control_land()
        elif self.dron.mode == "EXPLORE":
            return self.control_explore()
        elif self.dron.mode == "AVANZA":
            return self.control_avanza()
        elif self.dron.mode == "RETROCEDE":
            return self.control_retrocede()
        elif self.dron.mode == "APUNTA":
            return self.control_noviembre()
        # elif self.dron.mode == "NO_INIT":
        #     if self.mode == "CALIB_BIAS":
        #         return self.control_calib_bias()
        return()

    def init_planner(self):
        self.provisional_planner = 0
        self.provisional_planner_ts = datetime.now()


    def run_autoajuste(self, autoajustes):
        pass


    def swap_to_mode_hold(self, numero_ajustes):
        if self.dron.get_mode() == "AVANZA":
            recorder.elevatorRecord[-60:] = [self.dron.valor_maniobras] * 60
        self.ignore_derivative_error = True
        self.necesario_windup = True
        self.veces_autoajuste = self.max_veces_autoajuste-numero_ajustes
        self.autoadjust = datetime.now()
        self.dron.set_mode("HOLD")

    def swap_to_mode_avanza(self):
        pass

    def set_path_target_as_target(self):
        gb.xTarget = gb.path_x
        gb.yTarget = gb.path_y
        print("path point target is the new target:", str(gb.xTarget), str(gb.yTarget))


    def angulo_alcanzado(self):
        a_target = gb.angleTarget
        a_measured = gb.head
        if a_target >= 180: a_target -= 360
        if a_measured >= 180: a_measured -= 360
        if abs(a_target - a_measured) <= 5:
            return True
        return False


    ## Esperas...
    def inicia_espera(self, segundos):
        self.flag_esperando = True
        self.t_espera_caducidad = datetime.now() + timedelta(seconds=segundos)

    def get_flag_esperando(self):
        return self.flag_esperando

    def t_espera_vencido(self):
        if (datetime.now() >= self.t_espera_caducidad):
            return True
        return False

    def reset_espera(self):
        self.flag_esperando = False


    # Provisional, Logica que cambia modos de control - todo pasar a modo del dron...
    def run_meta_selector(self, windup=False):
        # print("[TEMPORAL: {} - {} - {}]".format(gb.z, controller.mode, gb.zTarget))
        # Force down

        # # Desarrollar un planner / navegador que lea de instrucciones de un fichero.
        # self.provisional_planner = (datetime.now() - self.provisional_planner_ts).seconds
        # if self.provisional_planner >= 12 and self.dron.mode == "HOLD":
        #     gb.angleTarget = rfs.calcula_angulo_en_punto(gb.path_x, gb.path_y, gb.x, gb.y)
        #     if
        #
        # # funcion objetivo dentro area
        # if self.dron.mode == "AVANZA":
        #     en_punto = False
        #     if gb.x <= gb
        #     if en_punto:
        #
        #         self.autoadjust = datetime.now()
        #         self.veces_autoajuste = self.max_veces_autoajuste - 1
        #         self.dron.set_mode("HOLD")

        if self.dron.mode != "NO_INIT":
            if self.dron.mode == "DESPEGUE":
                if sum(recorder.zRecord[-6:])/6 >= (gb.zTarget-5):
                    configurator.load_config_file("noviembre.json")
                    self.swap_to_mode_hold(2)

            if self.dron.mode == "APUNTA":
                gb.angleTarget = rfs.calcula_angulo_en_punto(gb.path_x, gb.path_y, gb.x, gb.y)
                if self.angulo_alcanzado():
                    rfs.pinta_circulo_target(4, macizo=True, color="verde")
                else:
                    rfs.pinta_circulo_target(4, macizo=True, color="rojo")
                if self.angulo_alcanzado() and not self.get_flag_esperando():
                    secs_espera = 2
                    print("Apuntando hacia punto objetivo, esperando " + str(secs_espera) + " segundos.")
                    self.inicia_espera(secs_espera)
                elif self.get_flag_esperando() and self.t_espera_vencido():
                    self.dron.set_mode("AVANZA")
                    self.reset_espera()

            elif self.dron.mode == "AVANZA":
                # si el dron esta en punto cercano al target: cambia amodo hold con swap_to_mode_hold
                gb.angleTarget = rfs.calcula_angulo_en_punto(gb.path_x, gb.path_y, gb.x, gb.y)
                if rfs.evalua_llegada_meta(55): # evalua_llegada_meta ya pinta circulo objetivo
                    self.set_path_target_as_target()
                    self.swap_to_mode_hold(1)

            if self.dron.mode == "HOLD":
                if self.veces_autoajuste < self.max_veces_autoajuste:
                    tiempo_desde_ultimo_ajuste = datetime.now() - self.autoadjust
                    if tiempo_desde_ultimo_ajuste >= timedelta(seconds=4):
                        if self.necesario_windup:
                            self.necesario_windup = False
                            self.windup()
                        self.veces_autoajuste += 1
                        print("ajustando todos ejes a la vez...")
                        configurator.load_config_file("media_todos_ejes.json")
                        # self.windup()  # chapuza windup
                        rango = 60
                        bias = {
                            "aileron": int(sum(recorder.aileronRecord[-rango:]) / rango),
                            "correccion_gravedad": 0,
                            "elevator": int(sum(recorder.elevatorRecord[-rango:]) / rango),
                            "rudder": 1500,     ## Probando
                            "throttle": int(sum(recorder.throttleRecord[-rango:]) / rango)
                        }
                        configurator.modify_bias(bias, "media_todos_ejes.json")
                        self.autoadjust = datetime.now()

            else:
                ## Provisionalmente esto no estará funcionando - 2018/12/03
                try:
                    if ((gb.z >= 60) and (self.mode == "BASICO" or self.mode == "BASICO_CLAMP_THROTTLE")):
                        self.consecutive_frames_out_of_limits += 1
                        if not self.forcing_down and self.consecutive_frames_out_of_limits >= 3:
                            print("FORCING DRON: A BAJAR!!!")
                            # midron.prepara_modo(timedelta(seconds=3), gb.throttle)
                            self.dron.set_mode("HOLD")
                            modo_panico = "BASICO_DISABLE_GFACTOR"
                            # controller.set_mode("BASICO_LIMIT_Z")
                            # controller.set_mode("BASICO_CLAMP_THROTTLE")
                            self.set_mode(modo_panico)
                            # self.windupZ()
                            self.forcing_down = True
                    # elif ((gb.z >= (gb.zTarget+20)) and (controller.mode == "BASICO" or controller.mode == "BASICO_CLAMP_THROTTLE" or controller.mode == modo_panico)):
                    #     consecutive_frames_out_of_limits += 1
                    else:
                        self.consecutive_frames_out_of_limits = 0

                    if self.forcing_down and gb.z <= gb.zTarget+10:
                        print("DESCATIVANDO FORCING. Activando control: BASICO")
                        self.forcing_down = False
                        if windup: self.windupZ()
                        self.set_mode("BASICO")
                        self.consecutive_frames_out_of_limits = 0
                except:
                    self.forcing_down = False
                    self.set_mode("BASICO")
                    self.consecutive_frames_out_of_limits = 0

    def control_con_filtros(self):
        # Filtros y PID control
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # record the position and orientation
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)

        # filter y
        buff = gb.fps
        if len(recorder.xRecord) >= buff:
            xFiltered = filtfilt(self.b, self.a, recorder.xRecord[-buff:])
            xDroneFiltered = xFiltered[-1]
            yFiltered = filtfilt(self.b, self.a, recorder.yRecord[-buff:])
            yDroneFiltered = yFiltered[-1]

            # filter z
            # zFiltered = lfilter(b, a, zRecord)
            zFiltered = filtfilt(self.b, self.a, recorder.zRecord[-buff:])
            zDroneFiltered = zFiltered[-1]

            # Tuneo medida leida angulo
            if self.anguloPrev:  # Previo y filtrado son el mismo valor. anguloPrevio y angleDroneFiltered
                while (angleDrone - self.anguloPrev) > 180:
                    angleDrone -= 360
                while (angleDrone - self.anguloPrev) <= -180:
                    angleDrone += 360

            # filter angle
            # angleFiltered = lfilter(b, a, angleRecord)
            angleFiltered = lfilter(self.ba, self.aa, recorder.angleMovidoRecord[-10:])    # a manota el 10
            angleDroneFiltered = angleFiltered[-1]
            self.anguloPrev = int(angleDroneFiltered)
                    # angleFilteredRecord.append(angleDroneFiltered)

            # El filtrado tuneado es el correcto, en rango 0-360 tras aplicar filtro
            angleDroneFilteredTuneado = int(self.anguloPrev)
            while angleDroneFilteredTuneado >= 360:
                angleDroneFilteredTuneado -= 360
            while angleDroneFilteredTuneado < 0:
                angleDroneFilteredTuneado += 360

            recorder.save_debug(angulo_movido=angleDrone, angulo_filtrado_raw=angleDroneFiltered)
            recorder.save_filtered_positions(xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado)

        else:
            recorder.save_debug(angulo_movido=angleDrone, angulo_filtrado_raw=angleDrone)
            recorder.save_filtered_positions(xDrone, yDrone, zDrone, angleDrone)
            xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado = xDrone, yDrone, zDrone, angleDrone
        # implement a PID controller

        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError

        if self.print_info: print("[LEIDO FILTRADO]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado))
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
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = self.zError - zError_old
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        zCommand = gb.KPz * zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

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
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle-gb.clamp_offset, gb.throttle_middle+gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle-gb.clamp_offset, gb.aileron_middle+gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle-gb.clamp_offset, gb.elevator_middle+gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle-gb.clamp_offset, gb.rudder_middle+gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # record everything
        recorder.save_errors(xError, yError, zError, angleError)
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)

    def control_basico(self):
        # Filtros y PID control
        gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # implement a PID controller
        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        self.xError = gb.xTarget - xDrone
        self.yError = gb.yTarget - yDrone
        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone

        # Trucar gravedad intento 1:
        if self.zError < 0 and gb.correccion_gravedad > 0: self.zError /= gb.correccion_gravedad
        # if zError < 0: print(zError)

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        if gb.KIz > 0:
            clamp_i = int((gb.clamp_offset / gb.KIz) * 0.90)  # :
            self.zErrorI = rfs.clamp(self.zErrorI, -clamp_i, clamp_i)


        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = self.zError - zError_old
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        zCommand = gb.KPz * self.zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        throttleCommand = gb.throttle_middle + zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand


        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # salva cosas
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        recorder.save_errors(self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI, modo="i")
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)

    def control_basico_pidzsimple(self):
        # Filtros y PID control
        gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # implement a PID controller
        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        self.xError = gb.xTarget - xDrone
        self.yError = gb.yTarget - yDrone
        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone


        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        if gb.KIz > 0:
            clamp_i = int((gb.clamp_offset / gb.KIz) * 0.90)  # :
            self.zErrorI = rfs.clamp(self.zErrorI, -clamp_i, clamp_i)

        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        zCommand = self.spid_z(self.zError)
        print(zCommand)
        print(zCommand)
        print(zCommand)
        throttleCommand = gb.throttle_middle - zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand

        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(
            rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(
            rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(
            rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(
            rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # salva cosas
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        # recorder.save_errors(self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI, modo="i")
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)

    def control_basico_factorsolointegral(self):
        # Filtros y PID control
        gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # implement a PID controller
        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        self.xError = gb.xTarget - xDrone
        self.yError = gb.yTarget - yDrone
        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone

        # Trucar gravedad intento 1:
        ziAux = self.zError
        zdAux = 1
        if self.zError < 0 and gb.correccion_gravedad > 0:
            ziAux = self.zError / gb.correccion_gravedad
            zdAux = gb.AUXILIAR1

        # if zError < 0: print(zError)

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += ziAux
        self.angleErrorI += self.angleError

        if gb.KIz > 0:
            clamp_i = int((gb.clamp_offset / gb.KIz) * 0.90)  # :
            self.zErrorI = rfs.clamp(self.zErrorI, -clamp_i, clamp_i)


        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = (self.zError - zError_old) * zdAux
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        zCommand = gb.KPz * self.zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        throttleCommand = gb.throttle_middle + zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand


        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # salva cosas
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        recorder.save_errors(self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI, modo="i")
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_basico_piddoble(self):
        # Filtros y PID control
        gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # implement a PID controller
        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        self.xError = gb.xTarget - xDrone
        self.yError = gb.yTarget - yDrone
        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone


        # if zError < 0: print(zError)

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        if gb.KIz > 0:
            clamp_i = int((gb.clamp_offset / gb.KIz) * 0.90)  # :
            self.zErrorI = rfs.clamp(self.zErrorI, -clamp_i, clamp_i)


        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = (self.zError - zError_old)
        angleErrorD = self.angleError - angleError_old

        if self.zError < 0 and gb.correccion_gravedad > 0:
            ziAux = self.zError / gb.correccion_gravedad
            zCommand = gb.KPzd * self.zError + gb.KIzd * ziAux + gb.KDzd * zErrorD
        else:
            zCommand = gb.KPz * self.zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        throttleCommand = gb.throttle_middle + zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand


        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # salva cosas
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        recorder.save_errors(self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI, modo="i")
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_basico_pruebas_filters(self):
        # PRUEBAS FILTROS OCTUBRE. Suavizar picos vision ayudará?

        # Filtros y PID control
        gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # record the position and orientation
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)

        buff = gb.fps
        if len(recorder.xRecord) >= buff:
            xDrone = lfilter(self.bbb, self.aaa, recorder.x[-buff:])  # a manota el 10
            yDrone = lfilter(self.bbb, self.aaa, recorder.y[-buff:])  # a manota el 10

        recorder.save_debug(angulo_movido=angleDrone, angulo_filtrado_raw=angleDroneFiltered)


        # implement a PID controller
        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError


        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        self.xError = gb.xTarget - xDrone
        self.yError = gb.yTarget - yDrone
        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone

        # Trucar gravedad intento 1:
        if self.zError < 0 and gb.correccion_gravedad > 0: self.zError /= gb.correccion_gravedad
        # if zError < 0: print(zError)

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = self.zError - zError_old
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        zCommand = gb.KPz * self.zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        throttleCommand = gb.throttle_middle + zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand

        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(
            rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(
            rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(
            rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(
            rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # salva cosas
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_pidlib_target(self, x=None, y=None, z=None, a=None):
        try:
            if x is not None: self.libpid_x.setpoint(x)
            if y is not None: self.libpid_y.setpoint(y)
            if z is not None: self.libpid_z.setpoint(z)
            if a is not None: self.libpid_a.setpoint(a)
        except:
            pass


    def control_pidlib_init(self):
        self.libpid_x = PID(gb.KPx, gb.KIx, gb.KDx, setpoint=gb.xTarget, output_limits=(1200, 1800), sample_time=None, proportional_on_measurement=True)
        self.libpid_y = PID(gb.KPy, gb.KIy, gb.KDy, setpoint=gb.yTarget, output_limits=(1200, 1800), sample_time=None, proportional_on_measurement=True)
        self.libpid_z = PID(gb.KPz, gb.KIz, gb.KDz, setpoint=gb.zTarget, output_limits=(1200, 1800), sample_time=None, proportional_on_measurement=True)
        self.libpid_a = PID(gb.KPangle, gb.KIangle, gb.KDangle, setpoint=180, output_limits=(1200, 1800), sample_time=None, proportional_on_measurement=True)


    def control_pidlib(self):
        # Filtros y PID control
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        throttleCommand = self.libpid_z(zDrone)
        elevatorCommand = self.libpid_y(yDrone)
        aileronCommand = self.libpid_x(xDrone)
        rudderCommand = self.libpid_a(angleDrone)

        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_simple_pid_init(self):
        self.spid_x = PID(gb.KPx, gb.KIx, gb.KDx, setpoint=gb.xTarget)
        self.spid_y = PID(gb.KPy, gb.KIy, gb.KDy, setpoint=gb.yTarget)
        self.spid_z = PID(gb.KPz, gb.KIz, gb.KDz, setpoint=0)
        self.spid_a = PID(gb.KPangle, gb.KIangle, gb.KDangle, setpoint=180)
        self.spid_x.output_limits = (-gb.clamp_offset, gb.clamp_offset)
        self.spid_y.output_limits = (-gb.clamp_offset, gb.clamp_offset)
        self.spid_z.output_limits = (-gb.clamp_offset, gb.clamp_offset)
        self.spid_a.output_limits = (-gb.clamp_offset, gb.clamp_offset)
        # self.spid_x = PID(gb.KPx, gb.KIx, gb.KDx, setpoint=gb.xTarget)
        # self.spid_y = PID(gb.KPy, gb.KIy, gb.KDy, setpoint=gb.yTarget)
        # self.spid_z = PID(gb.KPz, gb.KIz, gb.KDz, setpoint=gb.zTarget)
        # self.spid_a = PID(gb.KPangle, gb.KIangle, gb.KDangle, setpoint=180)
        # self.spid_x.output_limits = (-gb.clamp_offset, gb.clamp_offset)
        # self.spid_y.output_limits = (-gb.clamp_offset, gb.clamp_offset)
        # self.spid_z.output_limits = (-gb.clamp_offset, gb.clamp_offset)
        # self.spid_a.output_limits = (-gb.clamp_offset, gb.clamp_offset)



    def control_simple_pid(self):
        # Filtros y PID control
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # compute commands
        xCommand = self.spid_x(xDrone)
        yCommand = self.spid_y(yDrone)
        zCommand = self.spid_z(zDrone)
        angleCommand = self.spid_a(angleDrone)


        throttleCommand = gb.throttle_middle + zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand

        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_flip(self):
        tn = datetime.now()
        if tn - self.dron.t_start <= timedelta(seconds=0.1):
            command = "%i,%i,%i,%i,1000,2000" % (gb.throttle, gb.aileron, gb.elevator, gb.rudder)
            self.dron.send_command(command)
            return (gb.throttle, gb.aileron, gb.elevator, gb.rudder)
        elif tn - self.dron.t_start <= timedelta(seconds=0.15):
            command = "%i,%i,%i,%i,1000,1000" % (gb.throttle, gb.aileron, gb.elevator, gb.rudder)
            self.dron.send_command(command)
            return (gb.throttle, gb.aileron, gb.elevator, gb.rudder)
        elif tn - self.dron.t_start <= timedelta(seconds=0.20):
            return (gb.throttle, 1000, gb.elevator, gb.rudder)
        elif tn - self.dron.t_start <= timedelta(seconds=0.25):
            return (gb.throttle, self.dron.valor_maniobras, gb.elevator, gb.rudder)
        else:
            self.dron.set_mode(self.dron.modo_previo)
            return (gb.throttle, self.dron.valor_maniobras, gb.elevator, gb.rudder)

    def control_explore(self):
        tn = datetime.now()
        if tn - self.dron.t_start <= self.dron.t_duracion/2:
            return (gb.throttle+1, self.dron.valor_maniobras+150, gb.elevator, gb.rudder)
        elif tn - self.dron.t_start <= self.dron.t_duracion:
            return (gb.throttle, self.dron.valor_maniobras-150, gb.elevator, gb.rudder)
        else:
            self.dron.set_mode(self.dron.modo_previo)
            return (gb.throttle, self.dron.valor_maniobras, gb.elevator, gb.rudder)


    def control_land(self):
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        _, aileronCommand, elevatorCommand, rudderCommand = self.control_noviembre()

        tn = datetime.now()
        if tn - self.dron.t_start >= self.dron.t_duracion or zDrone <= 10:
            self.dron.set_mode("NO_INIT")
            throttleCommand = self.dron.valor_maniobras
            self.dron.panic()
            self.dron.flag_vuelo = False
        else:
            """
            vx será el valor del throttle
                tx - ti   vx - vi
                ------- = -------
                tf - ti   vf - vi
            """
            tx = tn
            ti = self.dron.t_start
            tf = self.dron.t_start + self.dron.t_duracion
            vi = self.dron.valor_maniobras
            vf = 1200
            vx = (tx - ti) / (tf - ti) * (vf - vi) + vi
            throttleCommand = vx


        # OLD LAND.
        # # compute commands
        # xCommand = self.spid_x(xDrone)
        # yCommand = self.spid_y(yDrone)
        # angleCommand = self.spid_a(angleDrone)
        #
        # elevatorCommand = gb.elevator_middle - yCommand
        # aileronCommand = gb.aileron_middle + xCommand
        # rudderCommand = gb.rudder_middle - angleCommand
        #
        # recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_calib_bias(self):
        # Filtros y PID control
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        gb.angleTarget = 180

        angleCommand = gb.KPangle * self.angleError

        t_ahora = datetime.now()

        elevatorCommand = gb.elevator_middle
        aileronCommand = gb.aileron_middle
        rudderCommand = gb.rudder_middle - angleCommand

        if t_ahora - self.t_inicio_maniobra <= timedelta(seconds=1):
            throttleCommand = 1500
        elif t_ahora - self.t_inicio_maniobra <= timedelta(seconds=3):
            throttleCommand = 1000
        else:
            throttleCommand = 1000
            e_x = gb.xTarget - gb.x
            e_y = gb.yTarget - gb.y
            print("Last Target:", gb.xTarget, gb.yTarget)
            print("Current Aileron / Elevator:", gb.aileron_middle, gb.elevator_middle)
            print("Errores:", e_x, e_y)
            if abs(e_x) >= 25 or abs(e_y) >= 25:
                if abs(e_x) >= 50:
                    gb.aileron_middle += (25 * np.sign(e_x))
                elif abs(e_x) >= 25:
                    gb.aileron_middle += (15 * np.sign(e_x))
                if abs(e_y) >= 25:
                    gb.elevator_middle -= (25 * np.sign(e_y))
                elif abs(e_y) >= 25:
                    gb.elevator_middle -= (15 * np.sign(e_y))
                self.t_inicio_maniobra = datetime.now()
                configurator.config_target(x=xDrone, y=yDrone)
                configurator.panels.refresh_sliders_from_globals()

            else:
                print("HEcho!")
                self.dron.set_mode("NO_INIT")
                self.set_mode("BASICO")
            print("New Aileron / Elevator:", gb.aileron_middle, gb.elevator_middle)

        self.angleError = gb.angleTarget - angleDrone

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))



        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_hover_bias(self):
        # Filtros y PID control
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        gb.angleTarget = 180

        zError_old = self.zError

        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone

        # Trucar gravedad intento 1:
        if self.zError < 0 and gb.correccion_gravedad > 0: self.zError /= gb.correccion_gravedad
        # if zError < 0: print(zError)

        self.zErrorI += self.zError

        zErrorD = self.zError - zError_old
        # a angulo no le vamos a meter I ni D, aqui al menos
        zCommand = gb.KPz * self.zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * self.angleError  # P

        throttleCommand = gb.throttle_middle + zCommand
        rudderCommand = gb.rudder_middle - angleCommand

        e_x = gb.xTarget - gb.x
        e_y = gb.yTarget - gb.y
        print("Errores:", e_x, e_y)
        if abs(e_x) >= 1:
            gb.aileron_middle += 5 * np.sign(e_x)
        if abs(e_y) >= 1:
            gb.elevator_middle -= 5 * np.sign(e_y)

        aileronCommand = gb.aileron_middle
        elevatorCommand = gb.elevator_middle

        configurator.config_target(x=xDrone, y=yDrone)
        configurator.panels.refresh_biases_from_globals()
        print("AIL / ELE / RUD: ", aileronCommand, elevatorCommand, rudderCommand)

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_basico_lfilter(self):
        # Filtros y PID control
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)

        # filter y
        buff = gb.fps
        if len(recorder.xRecord) >= buff:
            xFiltered = lfilter(self.b, self.a, recorder.xRecord[-buff:])
            xDroneFiltered = xFiltered[-1]
            yFiltered = lfilter(self.b, self.a, recorder.yRecord[-buff:])
            yDroneFiltered = yFiltered[-1]
            zFiltered = lfilter(self.b, self.a, recorder.zRecord[-buff:])
            zDroneFiltered = zFiltered[-1]
            angleFiltered = lfilter(self.b, self.a, recorder.angleRecord[-buff:])
            angleDroneFiltered = angleFiltered[-1]
        else:
            xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFiltered = xDrone, yDrone, zDrone, angleDrone

        # implement a PID controller
        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        self.xError = gb.xTarget - xDroneFiltered
        self.yError = gb.yTarget - yDroneFiltered
        self.zError = gb.zTarget + zDroneFiltered
        self.angleError = gb.angleTarget - angleDroneFiltered

        # Trucar gravedad intento 1:
        if self.zError < 0 and gb.correccion_gravedad > 0: self.zError /= gb.correccion_gravedad
        # if zError < 0: print(zError)

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = self.zError - zError_old
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        zCommand = gb.KPz * self.zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        throttleCommand = gb.throttle_middle + zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand


        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_sin_filtros(self):
        # Filtros y PID control
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # record the position and orientation
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)

        # filter y
        buff = gb.fps
        if len(recorder.xRecord) >= buff:
            xFiltered = filtfilt(self.b, self.a, recorder.xRecord[-buff:])
            xDroneFiltered = xFiltered[-1]
            yFiltered = filtfilt(self.b, self.a, recorder.yRecord[-buff:])
            yDroneFiltered = yFiltered[-1]

            # filter z
            # zFiltered = lfilter(b, a, zRecord)
            zFiltered = filtfilt(self.b, self.a, recorder.zRecord[-buff:])
            zDroneFiltered = zFiltered[-1]

            # Tuneo medida leida angulo
            if self.anguloPrev:  # Previo y filtrado son el mismo valor. anguloPrevio y angleDroneFiltered
                while (angleDrone - self.anguloPrev) > 180:
                    angleDrone -= 360
                while (angleDrone - self.anguloPrev) <= -180:
                    angleDrone += 360

            # filter angle
            # angleFiltered = lfilter(b, a, angleRecord)
            angleFiltered = lfilter(self.ba, self.aa, recorder.angleMovidoRecord[-10:])    # a manota el 10
            angleDroneFiltered = angleFiltered[-1]
            self.anguloPrev = int(angleDroneFiltered)
                    # angleFilteredRecord.append(angleDroneFiltered)

            # El filtrado tuneado es el correcto, en rango 0-360 tras aplicar filtro
            angleDroneFilteredTuneado = int(self.anguloPrev)
            while angleDroneFilteredTuneado >= 360:
                angleDroneFilteredTuneado -= 360
            while angleDroneFilteredTuneado < 0:
                angleDroneFilteredTuneado += 360

            recorder.save_debug(angulo_movido=angleDrone, angulo_filtrado_raw=angleDroneFiltered)
            recorder.save_filtered_positions(xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado)

        else:
            recorder.save_debug(angulo_movido=angleDrone, angulo_filtrado_raw=angleDrone)
            recorder.save_filtered_positions(xDrone, yDrone, zDrone, angleDrone)
            xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado = xDrone, yDrone, zDrone, angleDrone
        # implement a PID controller

        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError


        if self.print_info: print("[LEIDO FILTRADO]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xDroneFiltered, yDroneFiltered, zDroneFiltered, angleDroneFilteredTuneado))
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
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = self.zError - zError_old
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        zCommand = gb.KPz * zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

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
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle-gb.clamp_offset, gb.throttle_middle+gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle-gb.clamp_offset, gb.aileron_middle+gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle-gb.clamp_offset, gb.elevator_middle+gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle-gb.clamp_offset, gb.rudder_middle+gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # record everything
        recorder.save_errors(xError, yError, zError, angleError)
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)



    def control_clamp_throttle(self):
        # Filtros y PID control
        gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # implement a PID controller
        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        self.xError = gb.xTarget - xDrone
        self.yError = gb.yTarget - yDrone
        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone

        # # Trucar gravedad intento 1:
        # if self.zError < 0 and gb.correccion_gravedad > 0: self.zError /= gb.correccion_gravedad
        # # if zError < 0: print(zError)

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += 0
        self.angleErrorI += self.angleError

        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        # zErrorD = 0
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        zCommand = 1450
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        throttleCommand = 1450
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand


        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # salva cosas
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_disable_factor(self):
        # Filtros y PID control
        gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # implement a PID controller
        # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        # PRUEBAS CON FILTRO BUTTERWORTH - LIFTLFT - 0 phase
        self.xError = gb.xTarget - xDrone
        self.yError = gb.yTarget - yDrone
        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        if gb.KIz > 0:
            clamp_i = int((gb.clamp_offset / gb.KIz) * 0.80)
            self.zErrorI = rfs.clamp(self.zErrorI, -250, clamp_i)

        # ##########################################################################
        # ############ FORZAR ABAJO POR FACTOR GRAVEDAD A MANOTA
        # # Trucar gravedad intento 1:
        # if self.zError < 0 and gb.correccion_gravedad > 0:
        #     self.zError = self.zError / (gb.correccion_gravedad * 0.7)
        # # if zError < 0: print(zError)
        # ##########################################################################

        # Trucar gravedad intento 1:
        if self.zError < 0 and gb.correccion_gravedad > 0: self.zError /= gb.correccion_gravedad
        # if zError < 0: print(zError)

        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = self.zError - zError_old
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD
        zCommand = gb.KPz * self.zError + gb.KIz * self.zErrorI + gb.KDz * zErrorD
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        # print the X, Y, Z, angle commands
        # if info: print("[FILTER]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))
        # print("X:", str(xCommand))

        throttleCommand = gb.throttle_middle - zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand


        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # salva cosas
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        recorder.save_errors(self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI, modo="i")
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return(throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_noviembre_CHECKPOINT_181120(self):
        # cOPIA DEL control basico para pruebas en noviembre, tras 4 o 5 semanas sin tocar y no haber conseguido control OK con el nuevo filtro vision y su retardo.
        gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        self.xError = gb.xTarget - xDrone
        self.yError = gb.yTarget - yDrone
        self.zError = gb.zTarget - zDrone
        self.angleError = gb.angleTarget - angleDrone

        # Trucar gravedad intento 1:
        if self.zError < 0 and gb.correccion_gravedad > 0: self.zError /= gb.correccion_gravedad
        # if zError < 0: print(zError)

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        if gb.KIz > 0:
            clamp_i = int((gb.clamp_offset / gb.KIz) * 0.90)  # :
            self.zErrorI = rfs.clamp(self.zErrorI, -clamp_i, clamp_i)

        # compute derivative (variation) of errors (D)
        xErrorD = self.xError - xError_old
        yErrorD = self.yError - yError_old
        zErrorD = self.zError - zError_old
        angleErrorD = self.angleError - angleError_old

        # compute commands
        xCommand = gb.KPx * self.xError + gb.KIx * self.xErrorI + gb.KDx * xErrorD
        yCommand = gb.KPy * self.yError + gb.KIy * self.yErrorI + gb.KDy * yErrorD

        componente_ZP = gb.KPz * self.zError
        componente_ZI = gb.KIz * self.zErrorI
        componente_ZD = gb.KDz * zErrorD
        zCommand = componente_ZP + componente_ZI + componente_ZD

        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD

        throttleCommand = gb.throttle_middle + zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand

        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset, gb.rudder_middle + gb.clamp_offset))
        rudderCommand = round(rfs.clamp(rudderCommand, 1000, 2000))

        # print(componente_ZP, componente_ZI, componente_ZD)
        rfs.pinta_en_posicion([throttleCommand, componente_ZP, componente_ZI, componente_ZD], (int(xDrone), int(yDrone)))



        # salva cosas
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        recorder.save_errors(self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI, modo="i")
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())

        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def corrige_diferencia(self, diferencia, tiempo):
        # No es general, es válido para nuestro caso de 20fps
        if tiempo <= 0.025:
            return diferencia * 2
        elif tiempo >= 0.1:
            return diferencia / 2
        # valor = input * 10
        # multiplicador = valor * valor * (-7) + valor * 4 + 3.5
        # output = input * multiplicador
        divisor = tiempo * 20
        output = diferencia / divisor
        # print("provisional diferencia...", input, tiempo, output)
        return output


    def control_noviembre(self):
        # print("Entrando control_noviembre")
        # cOPIA DEL control basico para pruebas en noviembre, tras 4 o 5 semanas sin tocar y no haber conseguido control OK con el nuevo filtro vision y su retardo.
        # gb.angleTarget = 180
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head


        # ### Conversion X e Y para correccion aileron y elevator.
        # xDrone e yDrone ya no serian las posiciones leidas directamente de la vision?
        #      el error es lo que sería diferente y dependeria del angulo ...
        #      el error es la diferencia en pixeles  solamente porque el angulo era hasta ahora siempre 180
        #          pos objetivo - pos dron -> x e y directas
        #          en realidad eso seria igual que...
        #          angulo_de(pos objetivo-pos dron) * pos_seno(angulo dron) = x
        #          angulo_de(pos objetivo-pos dron) * pos_coseno(angulo dron) = y
        #          donde en el caso de dron a 180º coincide con las medidas directas de x e y
        #      por lo que...
        #          la medida del angulo nos va a afectar directamente a los valores de X e Y, eso me parece una mierda.
        #          si la visión no capta muy bien el ángulo en una afectaría a las otros 2 componentes.
        #
        #         igualmente.. vamos a probar

        distancia, angulo_movimiento = rfs.extrae_vector_posicion(xDrone, yDrone, gb.xTarget, gb.yTarget)
        angulo_giro = rfs.diferencia_angulos(angleDrone, angulo_movimiento)
        xErrorNuevo, yErrorNuevo = rfs.extrae_componentes(distancia, angulo_giro)

        # print("control!")
        # print(gb.frame_time - self.t_frame_previo)
        tiempo_entre_frames = (gb.frame_time - self.t_frame_previo)

        xError_old, yError_old, zError_old, angleError_old = self.xError, self.yError, self.zError, self.angleError
        self.xError = xErrorNuevo
        self.yError = yErrorNuevo
        self.zError = gb.zTarget - zDrone
        # self.angleError = gb.angleTarget - angleDrone     # cambiar por la resta mejorada
        self.angleError = rfs.diferencia_angulos(gb.angleTarget, angleDrone)

        # Trucar gravedad intento 1:
        if self.zError < 0 and gb.correccion_gravedad > 0: self.zError /= gb.correccion_gravedad
        # if zError < 0: print(zError)

        # compute integral (sum) of errors (I)
        # should design an anti windup for z
        self.xErrorI += self.xError
        self.yErrorI += self.yError
        self.zErrorI += self.zError
        self.angleErrorI += self.angleError

        # compute derivative (variation) of errors (D)
        if self.ignore_derivative_error:
            xErrorD, yErrorD, zErrorD, angleErrorD = 0, 0, 0, 0
            self.ignore_derivative_error = False
        else:
            xErrorD = self.xError - xError_old
            yErrorD = self.yError - yError_old
            zErrorD = self.zError - zError_old
            angleErrorD = self.angleError - angleError_old



        # compute commands
        ############################
        #     X
        componente_XP = gb.KPx * self.xError
        componente_XD = gb.KDx * self.corrige_diferencia(xErrorD, tiempo_entre_frames)
        # componente_XD = gb.KDx * xErrorD
        i, componente_XI = gb.KIx, 0
        if i:
            clamp_i = int((gb.clamp_offset / gb.KIx) * 0.65)
            self.xErrorI = rfs.clamp(self.xErrorI, -clamp_i, clamp_i)
            componente_XI = gb.KIx * self.xErrorI
        xCommand = componente_XP + componente_XI + componente_XD


        ############################
        #     Y
        componente_YP = gb.KPy * self.yError
        componente_YD = gb.KDy * self.corrige_diferencia(yErrorD, tiempo_entre_frames)
        # componente_YD = gb.KDy * yErrorD
        i, componente_YI = gb.KIy, 0
        if i:
            clamp_i = int((gb.clamp_offset / gb.KIy) * 0.65)
            self.yErrorI = rfs.clamp(self.yErrorI, -clamp_i, clamp_i)
            componente_YI = gb.KIy * self.yErrorI
        yCommand = componente_YP + componente_YI + componente_YD


        ############################
        #     A
        angleCommand = gb.KPangle * self.angleError + gb.KIangle * self.angleErrorI + gb.KDangle * angleErrorD


        ############################
        #     Z
        componente_ZP = gb.KPz * self.zError
        componente_ZD = gb.KDz * self.corrige_diferencia(zErrorD, tiempo_entre_frames)
        componente_ZD = rfs.clamp(componente_ZD, -200, 200)
        # componente_ZD = gb.KDz * zErrorD
        # Acompasar Clamp general con clam integral Z (No funciona bien por los golpes de la diferencia)
        i, componente_ZI = gb.KIz, 0
        if i:
            clamp_i = int((gb.clamp_offset / gb.KIz) * 0.90)
            v_max = gb.throttle_middle + gb.clamp_offset + 100 # margen de 100 throttle sobrepasados en la integrada
            provisional_zErrorI = rfs.clamp(self.zErrorI, -clamp_i, clamp_i)
            clamp_compas = v_max - componente_ZP - componente_ZD
            self.zErrorI = rfs.clamp(provisional_zErrorI, -clamp_i, clamp_compas)
        # componente_ZI = gb.KIz * self.zErrorI
            componente_ZI = gb.KIz * provisional_zErrorI
        zCommand = componente_ZP + componente_ZI + componente_ZD


        # generate rf values
        throttleCommand = gb.throttle_middle + zCommand
        elevatorCommand = gb.elevator_middle - yCommand
        aileronCommand = gb.aileron_middle + xCommand
        rudderCommand = gb.rudder_middle - angleCommand

        # round and clamp the commands to [1000, 2000] us (limits for PPM values)
        throttleCommand = round(rfs.clamp(throttleCommand, gb.throttle_middle - gb.clamp_offset, gb.throttle_middle + gb.clamp_offset))
        throttleCommand = round(rfs.clamp(throttleCommand, 1000, 2000))
        aileronCommand = round(rfs.clamp(aileronCommand, gb.aileron_middle - gb.clamp_offset, gb.aileron_middle + gb.clamp_offset))
        aileronCommand = round(rfs.clamp(aileronCommand, 1000, 2000))
        elevatorCommand = round(rfs.clamp(elevatorCommand, gb.elevator_middle - gb.clamp_offset, gb.elevator_middle + gb.clamp_offset))
        elevatorCommand = round(rfs.clamp(elevatorCommand, 1000, 2000))
        rudderCommand = round(rfs.clamp(rudderCommand, gb.rudder_middle - gb.clamp_offset/3, gb.rudder_middle + gb.clamp_offset/3))


        # print(componente_ZP, componente_ZI, componente_ZD)
        rfs.pinta_en_posicion([throttleCommand, componente_ZP, componente_ZI, componente_ZD], (int(xDrone), int(yDrone)))
        rfs.pinta_en_posicion(["X -", aileronCommand, componente_XP, componente_XI, componente_XD], (50, 240))
        rfs.pinta_en_posicion(["Y |", elevatorCommand, componente_YP, componente_YI, componente_YD], (550, 240))


        # salva cosas
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)
        recorder.save_errors(self.xError, self.yError, self.zError, self.angleError)
        recorder.save_errors(self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI, modo="i")
        recorder.save_commands(elevatorCommand, aileronCommand, throttleCommand, rudderCommand)
        recorder.save_time((datetime.now() - gb.timerStart).total_seconds())


        self.t_frame_previo = gb.frame_time
        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_avanza(self):
        (throttleCommand, aileronCommand, elevatorCommand, rudderCommand) = self.control_noviembre()
        elevatorCommand = self.dron.valor_maniobras
        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)


    def control_retrocede(self):
        (throttleCommand, aileronCommand, elevatorCommand, rudderCommand) = self.control_noviembre()
        elevatorCommand = self.dron.valor_maniobras
        return (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)

