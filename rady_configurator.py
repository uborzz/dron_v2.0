import json
import globals as gb
from datetime import datetime
import os
import cv2

class Panels():

    def __init__(self):
        self.create_trackbars()

    def create_trackbars(self):
        """
            globals:
            KPx, KPy, KPz, KPangle, KIx, KIy, KIz, KIangle, KDx, KDy, KDz, KDangle
            throttle_middle, rudder_middle, aileron_middle, elevator_middle, correccion_gravedad, clamp_offset
        """

        # target
        cv2.namedWindow('target')
        cv2.resizeWindow('target', 300, 120)
        cv2.createTrackbar('Z Target', 'target', 20, 50, self.nothing_target)
        cv2.createTrackbar('A Target', 'target', 180, 360, self.nothing_target)

        # trackbars
        cv2.namedWindow('control')
        cv2.resizeWindow('control', 400, 750)
        cv2.createTrackbar('KPZ', 'control', gb.KPz, 20, self.nothing)
        cv2.createTrackbar('KIZ', 'control', int(gb.KIz * 100), 100, self.nothing)
        cv2.createTrackbar('KDZ', 'control', gb.KDz, 200, self.nothing)
        cv2.createTrackbar('BIASZ', 'control', gb.throttle_middle, 2000, self.nothing)
        cv2.createTrackbar('CG', 'control', gb.correccion_gravedad, 25, self.nothing)

        cv2.createTrackbar('Clamp', 'control', gb.clamp_offset, 500, self.nothing)

        cv2.createTrackbar('KPX', 'control', int(gb.KPx * 100), 500, self.nothing)
        cv2.createTrackbar('KIX', 'control', int(gb.KIx * 100), 100, self.nothing)
        cv2.createTrackbar('KDX', 'control', gb.KDx, 200, self.nothing)
        cv2.createTrackbar('BIASAIL', 'control', gb.aileron_middle, 2000, self.nothing)

        cv2.createTrackbar('KPY', 'control', int(gb.KPy * 100), 500, self.nothing)
        cv2.createTrackbar('KIY', 'control', int(gb.KIy * 100), 100, self.nothing)
        cv2.createTrackbar('KDY', 'control', gb.KDy, 200, self.nothing)
        cv2.createTrackbar('BIASELE', 'control', gb.elevator_middle, 2000, self.nothing)

        cv2.createTrackbar('KPA', 'control', gb.KPangle, 20, self.nothing)
        cv2.createTrackbar('KIA', 'control', int(gb.KIangle * 100), 100, self.nothing)
        cv2.createTrackbar('KDA', 'control', gb.KDangle, 200, self.nothing)
        cv2.createTrackbar('BIASRUD', 'control', gb.rudder_middle, 2000, self.nothing)

    def refresh_sliders_from_globals(self):

        # trackbars
        cv2.setTrackbarPos('KPZ', 'control', gb.KPz)
        cv2.setTrackbarPos('KIZ', 'control', int(gb.KIz * 100))
        cv2.setTrackbarPos('KDZ', 'control', gb.KDz)
        cv2.setTrackbarPos('BIASZ', 'control', gb.throttle_middle)
        cv2.setTrackbarPos('CG', 'control', gb.correccion_gravedad)

        cv2.setTrackbarPos('Clamp', 'control', gb.clamp_offset)

        cv2.setTrackbarPos('KPX', 'control', int(gb.KPx * 100))
        cv2.setTrackbarPos('KIX', 'control', int(gb.KIx * 100))
        cv2.setTrackbarPos('KDX', 'control', gb.KDx)
        cv2.setTrackbarPos('BIASAIL', 'control', gb.aileron_middle)

        cv2.setTrackbarPos('KPY', 'control', int(gb.KPy * 100))
        cv2.setTrackbarPos('KIY', 'control', int(gb.KIy * 100))
        cv2.setTrackbarPos('KDY', 'control', gb.KDy)
        cv2.setTrackbarPos('BIASELE', 'control', gb.elevator_middle)

        cv2.setTrackbarPos('KPA', 'control', gb.KPangle)
        cv2.setTrackbarPos('KIA', 'control', int(gb.KIangle * 100))
        cv2.setTrackbarPos('KDA', 'control', gb.KDangle)
        cv2.setTrackbarPos('BIASRUD', 'control', gb.rudder_middle)


    def nothing(self, value):
        gb.refresca_params_flag = True


    def nothing_target(self, value):
        pass


    def refresca_params(self):
        gb.refresca_params_flag = False

        gb.KPz = cv2.getTrackbarPos("KPZ", "control")
        gb.KIz = cv2.getTrackbarPos("KIZ", "control") / float(100)
        gb.KDz = cv2.getTrackbarPos("KDZ", "control")
        gb.throttle_middle = cv2.getTrackbarPos("BIASZ", "control")
        gb.correccion_gravedad = cv2.getTrackbarPos("CG", "control")

        gb.clamp_offset = cv2.getTrackbarPos("Clamp", "control")

        gb.KPx = cv2.getTrackbarPos("KPX", "control") / float(100)
        gb.KIx = cv2.getTrackbarPos("KIX", "control") / float(100)
        gb.KDx = cv2.getTrackbarPos("KDX", "control")
        gb.aileron_middle = cv2.getTrackbarPos("BIASAIL", "control")

        gb.KPy = cv2.getTrackbarPos("KPY", "control") / float(100)
        gb.KIy = cv2.getTrackbarPos("KIY", "control") / float(100)
        gb.KDy = cv2.getTrackbarPos("KDY", "control")
        gb.elevator_middle = cv2.getTrackbarPos("BIASELE", "control")

        gb.KPangle = cv2.getTrackbarPos("KPA", "control")
        gb.KIangle = cv2.getTrackbarPos("KIA", "control") / float(100)
        gb.KDangle = cv2.getTrackbarPos("KDA", "control")
        gb.rudder_middle = cv2.getTrackbarPos("BIASRUD", "control")


class Configurator:

    # Gestiona sets de bias y params PID
    __instance = None

    def __new__(cls):
        if cls.__instance == None:
            cls.__instance = object.__new__(cls)
            cls.__instance.initialize("config_E011.json")  # config inicial
        return cls.__instance

    def initialize(self, file_name):
        self.salvar_al_salir = False
        with open("config/" + file_name, "r") as file:
            config_data = json.load(file)
        self.write_to_globals(config_data["pid"], config_data["bias"])
        gb.config_activa = file_name
        self.panels = Panels()

    def load_config_file(self, file_name):
        # Lee Config BIAS y PID
        with open("config/" + file_name, "r") as file:
            config_data = json.load(file)
        self.write_to_globals(config_data["pid"], config_data["bias"])
        self.panels.refresh_sliders_from_globals()
        gb.config_activa = file_name
        print(file_name, "cargada a globales.")

    @staticmethod
    def write_to_globals(pid_data, bias_data):
        # define middle PPM values that make the drone hover
        gb.throttle_middle = bias_data["throttle"]  # up (+) and down (-)  # 1660 con corona de 4 gramos
        gb.aileron_middle = bias_data["aileron"]  # left (-) and right (+)
        gb.elevator_middle = bias_data["elevator"]  # forward (+) and backward (-)
        gb.rudder_middle = bias_data["rudder"]  # yaw left (-) and yaw right (+)
        gb.correccion_gravedad = bias_data["correccion_gravedad"]

        gb.clamp_offset = 300

        # rady PID - pixeles (x-y) - cm (z) - degree (head)
        gb.KPx, gb.KPy, gb.KPz, gb.KPangle = pid_data["KPx"], pid_data["KPy"], pid_data["KPz"], pid_data["KPangle"]
        gb.KIx, gb.KIy, gb.KIz, gb.KIangle = pid_data["KIx"], pid_data["KIy"], pid_data["KIz"], pid_data["KIangle"]
        gb.KDx, gb.KDy, gb.KDz, gb.KDangle = pid_data["KDx"], pid_data["KDy"], pid_data["KDz"], pid_data["KDangle"]

    @staticmethod
    def save_config_file(file_name):
        config_data = dict()
        pid_data = {
            "KPx": gb.KPx, "KPy": gb.KPy, "KPz": gb.KPz, "KPangle": gb.KPangle,
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

        with open('config/' + file_name, 'w') as file:
            json.dump(config_data, file, sort_keys=True, indent=4)

    def save_to_config_activa(self):
        self.save_config_file(gb.config_activa)
        print("CONFIG SAVED!", gb.config_activa, "has been modified.")

    def load_config_activa(self):
        self.load_config_file(gb.config_activa)
        print(gb.config_activa, "config file LOADED!")

    def create_new_config_file(self):
        name = "{:%d_%H_%M}.json".format(datetime.now())
        self.save_config_file(name)
        print("NEW CONFIG FILE!", name, "has been created!")

    def select_another_config_file(self):
        file_list = os.listdir("config")
        index = file_list.index(gb.config_activa) + 1
        if index >= len(file_list):
            index = 0
        gb.config_activa = file_list[index]
        print(gb.config_activa, "config file has been selected! Press P to Load.")

    # Chapuza. Meter en paneles.
    def config_target(self, x=None, y=None, z=None, a=None):
        if z: cv2.setTrackbarPos('Z Target', 'target', z)
        if a: cv2.setTrackbarPos('A Target', 'target', a)



