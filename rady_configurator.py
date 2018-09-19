import json
import globals as gb
from datetime import datetime
import os

class Configurator:

    # Gestiona sets de bias y params PID
    __instance = None

    def __new__(cls):
        if cls.__instance == None:
            cls.__instance = object.__new__(cls)
            cls.__instance.load_config_file("config_E011.json")
        return cls.__instance

    def load_config_file(self, file_name):
        # Lee Config BIAS y PID
        with open("config/" + file_name, "r") as file:
            config_data = json.load(file)
        self.write_to_globals(config_data["pid"], config_data["bias"])
        gb.config_activa = file_name

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

    def load_config_activa(self):
        self.load_config_file(gb.config_activa)

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
        print(gb.config_activa, "config file has been selected!")