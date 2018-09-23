import rady_functions as rfs
import dron
from scipy.signal import butter, lfilter, filtfilt
import globals as gb
import numpy as np
import math
from datetime import datetime, timedelta
from rady_configurator import Configurator

recorder = rfs.Recorder()
configurator = Configurator()

class Controller():
    def __init__(self, info=False):
        self.print_info = info
        self.mode = "HOVER_BIAS"
        self.modes_available = ["BASICO", "BASICO_LF", "CALIB_BIAS", "HOVER_BIAS"]

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
        # provisional:
        self.anguloPrev = None
        self.xError, self.yError, self.zError, self.angleError = 0, 0, 0, 0
        self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI = 0, 0, 0, 0

        self.t_inicio_maniobra = datetime.now()

        self.dron = dron.take_dron()

    def change_mode(self):
        index = self.modes_available.index(self.mode) + 1
        if index >= len(self.modes_available):
            index = 0
        self.mode = self.modes_available[index]
        print("Modo controller elegido:", self.mode)

    def set_mode(self, mode):
        self.mode = mode

    def windup(self):
        self.xErrorI, self.yErrorI, self.zErrorI, self.angleErrorI = 0, 0, 0, 0

    def windupXY(self):
        self.xErrorI, self.yErrorI = 0, 0

    def control(self):
        if self.dron.mode == "DESPEGUE":
            if self.mode == "BASICO":
                return self.control_basico()
            elif self.mode == "HOVER_BIAS":
                return self.control_hover_bias()
            elif self.mode == "BASICO_LF": return self.control_basico_lfilter()
            elif self.mode == "CALIB_BIAS":
                return self.control_calib_bias()
            # return self.control_con_filtros()
        elif self.dron.mode == "HOLD":
            return self.control_con_filtros()
        # elif self.dron.mode == "NO_INIT":
        #     if self.mode == "CALIB_BIAS":
        #         return self.control_calib_bias()
            return()


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
        self.zError = gb.zTarget - zDroneFiltered
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