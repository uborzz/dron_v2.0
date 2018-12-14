# -*- coding: utf-8 -*-

# vars config especificas de la maquina/lugar

arduino_port =  "COM9"  # "/dev/ttyUSB0"    # "COM6" zum-bt, "COM7" nano.
camera_src = 1                  # 0                 # 1
camera_rotation = True          # False             # True
entorno = "prod"                # prod              # labo
altura_camara = 185 # cms       # 180               # 205

ignore_arduino = False # evita serial.open y intentos transf. data

pid_config_inicial = "29_14_04.json"