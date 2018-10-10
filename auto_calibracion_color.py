# -*- coding: utf-8 -*-

## Por hacer...
#   Autoajuste params camara segun luz y autoajuste valores hsv para localización

import cv2
import numpy as np
import rady_locator

def data_from_frame(img):
    # funcion recibe una imagen, extrae info numerica de ella y devuelve dicha info
    pass

def compara_data(data1, data2):
    # devuelve cuantificada la diferencia entre las 2 imagenes. ¿Como de parecidas son?
    pass

def ajustes_params_driver():
    """
    Toma una imagen nueva
    data_from_frame imagen nueva
    compara_data con data de imagen almacenada
    sube parametros camara
    repetir si mejora, deshacer si empeora

    ** Afinar un poco de esta forma contraste, brillo y saturacion camara.
        varias iteraciones.

    """
    pass

def ajustes_HSV_values():
    """
    Recibe imagen maskara dron
        se debe ver un contorno de tamaño min < X < max

    Se parte de valores previos aprox en locator.

    Se ve OK?

    Subir minimos hasta no VER -> Marca minimo -> set -4

    Bajar maximo hasta no VER -> Marca maximo. -> set +4

    """
    pass

# Funcion que realizará las movidas, para ello recibirá el stream de video y el localizador.
def main(locator, stream):

localizador = rady_locator.Localizador()

# Lee almacenada, por ejemplo.
img = cv2.imread("captures\ss_1001093433.jpg")
cv2.imshow("imagen", img)
cv2.waitKey()
