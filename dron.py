# -*- coding: utf-8 -*-

import time
from datetime import datetime, timedelta
import serial
import globales as gb
from rady_configurator import Configurator
from rady_functions import Recorder

configurator = Configurator()
recorder = Recorder()

class Dron:
    """
        Dron no se esta usando como representacion del propio dron.
        Se estan usando globales que recogen la info del localizador para representar
            la posicion del dron -de momento-
        La comunicacion serial sale a traves de esta clase.
            Los comandos/acciones enviados al dron estan programados aqui.
            El control está en rady_controller
    """

    def __init__(self, serial_port, simulated=False):
        self.baud_rate = 115200
        self.serial_port_name = serial_port
        self.port = None
        self.simulated = simulated
        if not self.simulated:
            self.port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)

        self.modes_available=["NO_INIT", "CALIB_COLOR", "DESPEGUE", "HOVER", "HOLD", "AVANZA", "RETROCEDE"]
        self.mode = "NO_INIT"
        self.motor_on = False
        self.drone_properties = "drone.config"

        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.pos_head = 0

        # self.x_pid = PID.PID()
        # self.y_pid = PID.PID()
        # self.z_pid = PID.PID()

        self.prueba_arduino_envios = 0
        self.last_command_received = "1000,1000,1000,1000"

        # thread = threading.Thread(target=self.read_from_port)
        # thread.start()
        self.flag_vuelo = False
        self.t_start = datetime.now()
        self.t_duracion = timedelta(seconds=1)
        self.valor_maniobras = 0
        self.modo_previo = None

    # def activar(self):
    #     while True:

    def restart(self):
        if not self.simulated:
            self.port.close()
            time.sleep(0.5)
            self.port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            time.sleep(0.5)
            print("Port restarted!")

    def get_port(self):
        return(self.port)

    # def read_from_port(self):
    #     while True:
    #         if self.port.
    #         # print("test")
    #         reading = self.port.readline().decode()
    #         print(reading)

    def set_position(self, x, y, z, head):
        self.pos_x = x
        self.pos_y = y
        self.pos_z = z
        self.pos_head = head

    def set_target(self, x, y):
        self.target_x = x
        self.target_y = y

    def set_altitude(self, z):
        self.target_z = z

    def set_heading(self, head):
        self.target_head = head

    def get_mode(self):
        return self.mode

    def activar_modo_previo(self):
        self.set_mode(self.modo_previo)

    def set_mode(self, modo):
        self.modo_previo = self.mode
        self.mode = modo
        print("Activando modo dron:", self.mode)
        if modo == "DESPEGUE":
            gb.xTarget, gb.yTarget, gb.angleTarget = gb.x, gb.y, gb.head
            configurator.config_target(a=gb.head)
        elif modo == "HOLD":
            gb.xTarget, gb.yTarget, gb.angleTarget = gb.x, gb.y, gb.head
            configurator.config_target(a=gb.head)
        elif modo == "AVANZA":
            l = recorder.elevatorRecord[-60:]
            self.valor_maniobras = sum(l) / len(l) - 50
        elif modo == "RETROCEDE":
            l = recorder.elevatorRecord[-60:]
            self.valor_maniobras = sum(l) / len(l) + 60
        # elif modo == "HOLD":
        #     gb.xTarget, gb.yTarget, gb.zTarget, gb.angleTarget = gb.x, gb.y, gb.z, gb.head
        #     configurator.config_target(z=gb.z, a=gb.head)


    def change_mode(self):
        index = self.modes_available.index(self.mode) + 1
        if index >= len(self.modes_available):
            index = 0
        self.set_mode(self.modes_available[index])
        print("Modo dron elegido:", self.mode)

    def turn_motors_ON(self):
        # self.z_pid.reset()
        # self.y_pid.reset()
        # self.x_pid.reset()
        self.motor_on = True
        pass

    def turn_motors_OFF(self):
        self.motor_on = False
        # value update?

    def prepara_modo(self, duracion, valor):
        self.t_start = datetime.now()
        self.t_duracion = duracion
        self.valor_maniobras = int(valor)
        self.modo_previo = self.mode

    #
    # def calibrate(self):
    #     command = "0,1000,1000,500"
    #     self.send_command(command)
    #
    # def calibrate2(self):
    #     command = "2000,1500,1500,1500"
    #     self.send_command(command)
    #     time.sleep(0.3)
    #     command = "1000,1500,1500,1500"
    #     self.send_command(command)
    #     time.sleep(0.3)

    def abajoizquierda(self):
        command = "1000,1000,1000,1000"
        self.send_command(command)


    def abajoderecha(self):
        command = "1000,2000,1000,2000"
        self.send_command(command)

    def neutro(self):
        command = "1000,1500,1500,1500"
        self.send_command(command)

    # BWHOOP == abajoderecha
    def calibrate(self):
        command = "1000,2000,1000,2000"
        self.send_command(command)

    # BWHOOP
    def calibrate2(self):
        command = "1000,2000,1000,2000"
        self.send_command(command)
        time.sleep(0.3)
        command = "1000,1000,1000,1000"
        self.send_command(command)
        time.sleep(0.3)
        command = "1000,1500,1500,1500"
        self.send_command(command)

    def envia_canal_valor(self, canal, valor):
        lista = [0] * canal
        lista[canal-1] = valor
        command = ",".join(map(str,lista))
        command.rstrip(',')
        # print(command)
        self.write(command)

    def send_command(self, command):
        res = command + "\n"
        self.write(res)


    def set_command(self, command):
        self.last_command_received = command

    def writeln(self, text):
        self.write(text + "\r\n")

    def write(self, text):
        if not self.simulated:
            e = self.port.write(text.encode('ascii'))
        # serial_line = self.port.read(2000)
        # print(serial_line)
        self.prueba_arduino_envios += 1
        # ts = time.time()
        # print(ts)

    def control(self):
        # if self.motor_on:
        #     if self.mode == "HOVER":
        #         self.z_pid.update(self.pos_z)
        #         # self.y_pid.update(self.pos_y)
        #         # self.x_pid.update(self.pos_x)
        pass

    def panic(self):
        command = "1000,1000,1000,1000"
        self.send_command(command)
        self.send_command(command)
        self.send_command(command)
        self.send_command(command)

    def close(self):
        self.port.close()
        self.port = serial.Serial(self.serial_port_name, 115200, timeout=.01)
        self.port.close()

def create_dron(port, simulated=False):
    global midron
    midron = Dron(port, simulated)
    return take_dron()

def take_dron():
    global midron
    return midron

