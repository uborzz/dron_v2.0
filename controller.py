import rady_functions as rfs
from scipy.signal import butter, lfilter, filtfilt
import globals as gb
import numpy as np
import math
from datetime import datetime

recorder = rfs.Recorder()

class Controller():
    def __init__(self, info=False):
        self.print_info = True

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

    def control(self):
        xDrone, yDrone, zDrone, angleDrone = gb.x, gb.y, gb.z, gb.head

        # ##########################
        # Filtros y PID control

        # record the position and orientation
        recorder.save_position(xDrone, yDrone, zDrone, angleDrone)

        # filter y
        buff = gb.fps
        if len(recorder.xRecord) >= buff:
            xFiltered = filtfilt(self.b, self.a, recorder.xRecord[-buff:])
            xDroneFiltered = xFiltered[-1]
                    # xFilteredRecord.append(xDroneFiltered)
            yFiltered = filtfilt(self.b, self.a, recorder.yRecord[-buff:])
            yDroneFiltered = yFiltered[-1]
                    # yFilteredRecord.append(yDroneFiltered)

            # filter z
            # zFiltered = lfilter(b, a, zRecord)
            zFiltered = filtfilt(self.b, self.a, recorder.zRecord[-buff:])
            zDroneFiltered = zFiltered[-1]
                    # zFilteredRecord.append(zDroneFiltered)

            # Tuneo medida leida angulo
            if self.anguloPrev:  # Previo y filtrado son el mismo valor. anguloPrevio y angleDroneFiltered
                while (angleDrone - self.anguloPrev) > 180:
                    angleDrone -= 360
                while (angleDrone - self.anguloPrev) <= -180:
                    angleDrone += 360

                    # angleMovidoRecord.append(angleDrone)

            # filter angle
            # angleFiltered = lfilter(b, a, angleRecord)
            print(recorder.angleMovidoRecord)
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