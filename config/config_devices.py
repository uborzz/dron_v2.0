# -*- coding: utf-8 -*-

# vars config especificas de la maquina/lugar

arduino_port =  "COM7"  # "/dev/ttyUSB0"    # "COM6" zum-bt, "COM7" nano.
camera_src = 1                  # 0                 # 1
camera_rotation = True          # False             # True
entorno = "prod"                # prod              # labo
altura_camara = 190 # cms       # 185               # 205
camera_undistort = True

ignore_arduino = False # evita serial.open y intentos transf. data
# rady_correction = None
locator_3points = True

pid_config_inicial = "29_14_04.json"

# todo: localización inicial por aruco o similar
posicion_estimada_inicial = (320, 240) # x, y
radio_zona_estimada_inicial = 80   # pixeles