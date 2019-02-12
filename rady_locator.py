# -*- coding: utf-8 -*-

### LOCATOR
# Clases para localización del Dron
# . Circulo: Blobdetector que busca un punto de valor HSV comprendido entre unos umbrales
# . Corona: Buscador del mayor contorno de valor HSV comprendido entre unos umbrales
# . Localizador: instancia 1x circulo y 1x corona y estima a partir de ellos la orientacion del dron
# Las clases anteriores tienen filtros por cercanía de localización anterior y suavizado por kalman.
# . HSVRangePanels: clase que pueden utilizar Circulo y Corona para implementar un panel con sliders de sus parametros.
#
# - rady

import sys
import cv2
import numpy as np
import time
import queue
import kalman_jc as kf
import kalman_circular as kfc
import math
import globales as gb
import rady_functions as rfs
import config.config_devices as cfg

def kernel_cuadrado(n):
    return np.ones((n, n), np.uint8)

def kernel_circular(n):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))


class Localizador:
    # Tendra 2 bolas y una corona - Corona solo
    def __init__(self, distancia_camara_suelo, debug=False, info=False, kalman_angle=True, entorno="Labo"):
        self.debug = debug
        self.info = info
        self.loc_3points = cfg.locator_3points

        diametro_corona = 12

        # LABO
        entorno = entorno.lower()
        if entorno == "labo":
            lower_corona = np.array([20, 50, 50])
            upper_corona = np.array([30, 255, 255])     ## <- Naranja ~ 20-25
            lower_punto = np.array([38, 50, 50])
            upper_punto = np.array([52, 255, 255])
            diametro_corona = 13.1

        # PROD - PROVISIONA FICHEROS
        elif entorno == "prod":
            try:    # si existen ficheros los carga
                lower_corona = np.loadtxt('config/config_locator/lower_corona.txt', dtype=int)
                upper_corona = np.loadtxt('config/config_locator/upper_corona.txt', dtype=int)
                lower_punto = np.loadtxt('config/config_locator/lower_punto.txt', dtype=int)
                upper_punto = np.loadtxt('config/config_locator/upper_punto.txt', dtype=int)
                lower_puntoD = np.loadtxt('config/config_locator/lower_puntoD.txt', dtype=int)
                upper_puntoD = np.loadtxt('config/config_locator/upper_puntoD.txt', dtype=int)
                lower_puntoI = np.loadtxt('config/config_locator/lower_puntoI.txt', dtype=int)
                upper_puntoI = np.loadtxt('config/config_locator/upper_puntoI.txt', dtype=int)
            except: # Nota!! Para guardar los ficheros salir con Q! Esc no guarda nada.
                lower_corona = np.array([16, 234, 246])
                upper_corona = np.array([24, 255, 255])
                lower_punto = np.array([137, 0, 168])  # LILA
                upper_punto = np.array([162, 172, 255])
            diametro_corona = 13.1


        elif entorno == "prod_saturado":  # Por ajustar.
            lower_corona = np.array([7, 220, 220])
            upper_corona = np.array([16, 255, 255])
            # lower_punto = np.array([56, 255, 255]) # TRUE VERDE ORIGINAL
            # upper_punto = np.array([63, 255, 255])
            lower_punto = np.array([152, 230, 230])  # LILA
            upper_punto = np.array([175, 255, 255])
            # lower_punto = np.array([95, 255, 255])  # AZUL SUBRAYADOR
            # upper_punto = np.array([110, 255, 255])

        # CASA
        elif entorno == "casa":
            lower_corona = np.array([5, 90, 140])
            upper_corona = np.array([25, 200, 200])
            lower_punto = np.array([34, 125, 120])
            upper_punto = np.array([45, 255, 255])

        # NEGRO
        elif entorno == "negro":
            lower_corona = np.array([70, 0, 0])
            upper_corona = np.array([110, 100, 100])
            lower_punto = np.array([34, 125, 120])
            upper_punto = np.array([45, 255, 255])

        else:
            lower_corona = np.array([20, 50, 50])
            upper_corona = np.array([30, 255, 255])
            lower_punto = np.array([38, 50, 50])
            upper_punto = np.array([52, 255, 255])

        self.distancia_camara_suelo = distancia_camara_suelo
        # self.coronaNaranja = Corona(lower_corona, upper_corona, 3, 13.1, debug=self.debug)  # <-- Corona negra 13.1
        self.coronaNaranja = Corona(lower_corona, upper_corona, 3, diametro_corona, debug=self.debug)    # <-- Corona naranja habitual. Espera reduccion peso.
        # self.coronaNaranja = Corona(lower_rojo, upper_rojo, 3, tamano_real=11, debug=self.debug)      # <-- Carcasa Dron Rojo prueba.
        self.circuloColor = punto_contorno(lower_punto, upper_punto, id_panel="verde F", angulo_real=0, trys=3, debug=self.debug)
        self.circuloColorI = punto_contorno(lower_puntoI, upper_puntoI, id_panel="azul I", angulo_real=-90, trys=3, debug=self.debug)
        self.circuloColorD = punto_contorno(lower_puntoD, upper_puntoD, id_panel="rojo D", angulo_real=90, trys=3, debug=self.debug)

        # self.K_x = kf.KalmanFilter(1, 1)
        # self.K_y = kf.KalmanFilter(1, 1)
        # self.K_head = kf.KalmanFilter(1, 1)
        # self.K_z = kf.KalmanFilter(1, 1)

        self.head = 0
        self.previous_heads = rfs.rady_ring(3)
        # self.K_head = kfc.kalman_circular(0.5, 30) # prueba filtro
        self.K_head = kfc.kalman_circular(4, 6)


        #filtro ruido
        self.angulo_filtra_ruido_veces = 0
        self.angulo_filtra_ruido_max_veces_consecutivas = 2

        # self.coronaNaranja = Corona(2)
        pass

    ################ PROVISIONAL #######################
    def save_values_to_file(self):
        np.savetxt('config/config_locator/upper_corona.txt', self.coronaNaranja.upper_limit, fmt='%d')
        np.savetxt('config/config_locator/lower_corona.txt', self.coronaNaranja.lower_limit, fmt='%d')
        np.savetxt('config/config_locator/upper_punto.txt', self.circuloColor.upper_limit, fmt='%d')
        np.savetxt('config/config_locator/lower_punto.txt', self.circuloColor.lower_limit, fmt='%d')
        np.savetxt('config/config_locator/upper_puntoD.txt', self.circuloColorD.upper_limit, fmt='%d')
        np.savetxt('config/config_locator/lower_puntoD.txt', self.circuloColorD.lower_limit, fmt='%d')
        np.savetxt('config/config_locator/upper_puntoI.txt', self.circuloColorI.upper_limit, fmt='%d')
        np.savetxt('config/config_locator/lower_puntoI.txt', self.circuloColorI.lower_limit, fmt='%d')
        print("Saving locator values to files...")

    ####################################################


    def value_to_debug_images(self, valor):
        self.debug = valor
        self.circuloColor.debug = valor
        self.circuloColorD.debug = valor
        self.circuloColorI.debug = valor
        self.coronaNaranja.debug = valor
        if not valor:
            cv2.destroyWindow("mascara")
            cv2.destroyWindow("filtrado")
            cv2.destroyWindow("mascara1")
            cv2.destroyWindow("mascara2")
            cv2.destroyWindow("circulo_dilation")
            cv2.destroyWindow("circulo focalizada externa")
            cv2.destroyWindow("RAW")
            cv2.destroyWindow("after umbral")
            cv2.destroyWindow("substract")

    def toggle_locator_3points(self):
        self.loc_3points = not self.loc_3points
        print("locator 3 points set to {}", str(self.loc_3points))

    def toggle_debug_images(self):
        valor = not self.debug
        gb.aux_debug = valor
        self.value_to_debug_images(valor)
        print("debug filter images to {}", str(valor))

    # define a clamping function
    def clamp(self, n, minimum, maximum):
        return max(min(maximum, n), minimum)

    class EstimadorAltura():

        anchura_real = 130 # mm
        distancia_focal = 2.8 # mm
        anchura_sensor = 3.58 # mm
        anchura_imagen = 640 # pixeles
        correccion_a_manuti = 1.14 # Quizas no están 100% ok los valores, asi que hemos ajustado con un factor a mano

        @classmethod
        def calcula_distancia(cls, vista_objeto):
            # print("radio:", str(vista_objeto))
            distancia = rfs.safe_division( cls.anchura_real * cls.distancia_focal * cls.anchura_imagen * cls.correccion_a_manuti,
                                           vista_objeto * cls.anchura_sensor ) / 10 # paso a cm
            altura = cfg.altura_camara - distancia
            return rfs.clamp(altura, 0, 70)

    def estima_altura_aproximacion(self, vista_objeto, x, y, hmax=200, corona=13):
        # a mano para corona definitiva de 13.
        distancia_al_medio = math.sqrt((x-320)**2 + (y-240)**2)
        distancia_al_medio = rfs.clamp(distancia_al_medio, 0, 210) # tomamos 210 como nuestro maximo lejos centro.
        factor_distancia = 1 - distancia_al_medio / 1120
        # print(vista_objeto)

        # pruebas.
        if vista_objeto:
            dd = (6 * 13 * 640) / (vista_objeto * 3)
            print(dd)

        if vista_objeto:
            # aprox para 14 cm - 10945
            dis = -8.84 * vista_objeto + 617.84
            dis = dis * factor_distancia
            res = rfs.clamp(hmax-dis, 0, 140)
            res /= 2
            return(res)
        else:
            return 0

    def estima_altura_lente_normal(self, vista_objeto, hmax=200, corona=13):
        # corona es el tamano real en cm de la corona que buscamos
        # Altura = 200 * (1 - (Do / objeto)) # H = 200
        # print("objeto: ", str(objeto))
        if vista_objeto:
            # aprox para 14 cm - 10945
            res = hmax - 10945 * (corona / 13) / vista_objeto  # Max distancia 200 cm.
            res = rfs.clamp(res, 0, 70)
            return (res)
        else:
            return 0


    def posicion_dron(self, frame):
        # Aplicar filtro Kalman

        # resized_frame = cv2.resize(frame, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_CUBIC)
        # frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # self.circuloAmarillo.find(frameHSV) # encuentra punto amarillo
        mask_corona = self.coronaNaranja.find(frame)  # encuentra corona
        # Cambio 2019/01/28 pruebas filtro dentro de corona.
        # self.circuloColor.find(frame, roi_user=self.coronaNaranja.mask_zona)  # encuentra punto verde

        x = self.coronaNaranja.I_x
        y = self.coronaNaranja.I_y

        # comentado, ya hay un kalman en la corona.
        # x = self.K_x.predict_and_correct(x)
        # y = self.K_x.predict_and_correct(y)
        # z = self.tamano_maximo_corona()

        # Ajuste Z
        # z = self.estima_altura(self.coronaNaranja.radius * 2, x, y, hmax=self.distancia_camara_suelo, corona=self.coronaNaranja.tamano_real_cm) # referencia a radio del circulo de la corona
        z = self.EstimadorAltura.calcula_distancia(self.coronaNaranja.radius*2) # referencia a radio del circulo de la corona

        # Tema angulo - posicion 3 puntos.
        if self.loc_3points:
            traza = False

            self.circuloColor.find(frame, roi_user=mask_corona)  # encuentra punto central (verde)
            self.circuloColorD.find(frame, roi_user=mask_corona)    # encuentra punto derecha
            self.circuloColorI.find(frame, roi_user=mask_corona)    # encuentra punto izquierda

            status1 = status2 = status3 = True
            head1 = head2 = head3 = -1
            if gb.solo_buscar_en_cercanias and self.circuloColor.last_located >= 1:
                status1 = False
            else:
                orientacion = math.atan2(x - self.circuloColor.I_x, y - self.circuloColor.I_y)  # radianes
                orientacion = int(math.degrees(orientacion))    # grados (-180, 180)
                orientacion += 180 # cambio a nuestro sistema de cordenadas.
                if traza: print("CENTRAL VECTOR", orientacion)
                head1 = rfs.suma_angulos(orientacion, self.circuloColor.angulo_real, range_0_360=True)
                if traza: print("CENTRAL HEAD", head1)
            if gb.solo_buscar_en_cercanias and self.circuloColorI.last_located >= 1:
                status2 = False
            else:
                orientacion = math.atan2(x - self.circuloColorI.I_x, y - self.circuloColorI.I_y)  # radianes
                orientacion = int(math.degrees(orientacion)) + 180
                if traza: print("IZQUIERDA VECTOR", orientacion)
                head2 = rfs.suma_angulos(orientacion, self.circuloColorI.angulo_real, range_0_360=True) # rango diff [-180, 180) + 180 -> [0, 360)
                if traza: print("IZQUIERDA HEAD", head2)
            if gb.solo_buscar_en_cercanias and self.circuloColorD.last_located >= 1:
                status3 = False
            else:
                orientacion = math.atan2(x - self.circuloColorD.I_x, y - self.circuloColorD.I_y)  # radianes
                orientacion = int(math.degrees(orientacion)) + 180
                if traza: print("DERECHA VECTOR", orientacion)
                head3 = rfs.suma_angulos(orientacion, self.circuloColorD.angulo_real, True)# revisar dif angulos.
                if traza: print("DERECHA HEAD", head3)

            status = True
            # print(head1, head2, head3)
            if (status1 and status2 and status3):
                head = rfs.meanangle([head1, head2, head3])
            elif (status1 and status2):
                head = rfs.meanangle([head1, head2])
            elif (status2 and status3):
                head = rfs.meanangle([head2, head3])
            elif (status1 and status3):
                head = rfs.meanangle([head1, head3])
            else:
                head = self.head
                status = False

        else:
            self.circuloColor.find(frame, roi_user=mask_corona)  # encuentra punto central
            if gb.solo_buscar_en_cercanias and self.circuloColor.last_located >= 1:
                head = self.head
                status = False
            else:
                head = math.atan2(x-self.circuloColor.I_x, y-self.circuloColor.I_y) # radianes
                head = int(math.degrees(head)) + 180

            if self.debug: print("circuloLeido: ", self.circuloColor.x, self.circuloColor.y)


        ## 2018/12/10
        ## PROBANDO ANTES DEL FILTRO DE KALMAN.
        head_mean = self.previous_heads.meanangles()
        if abs(head - head_mean) >= 55 and self.angulo_filtra_ruido_veces < self.angulo_filtra_ruido_max_veces_consecutivas:
            head = self.previous_heads.get_last() # No acaba de molar. La media tampoco.
            self.angulo_filtra_ruido_veces += 1
            # status = False
        else:
            self.angulo_filtra_ruido_veces = 0

        # filtrar kalman
        if not gb.disable_all_kalmans and gb.kalman_angle:
            if self.debug: print("Antes kalman circular:", head)
            head = self.K_head.predict_and_correct(head)
            if self.debug: print("Despues kalman circular:", head)

        # ## 2018/12/04
        # ## excluir medida? hacer esta historia general??:
        # head_mean = self.previous_heads.meanangles()
        # if abs(head - head_mean) >= 50:
        #     head = rfs.meanangle([head, head_mean])  # en cierto modo le damos peso a la media anterior y anadimos el nuevo valor.

        self.previous_heads.append(head)
        self.head = head

        # Para dibujar la flecha direccion
        x2 = int(x - self.coronaNaranja.radius * math.sin(math.radians(head-180)))
        y2 = int(y - self.coronaNaranja.radius * math.cos(math.radians(head-180)))

        if self.info: print("[LOCALIZADOR]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(x, y, z, head))

        cv2.arrowedLine(frame, (x,y), (x2, y2), (50,255,230), 2)


        # DRAW:
        # cv2.circle(frame, (self.circuloColor.I_x, self.circuloColor.I_y), 4, (0,255,0), thickness=-1)
        # cv2.circle(frame, (self.circuloAmarillo.I_x, self.circuloAmarillo.I_y), 4, (0, 255, 255), thickness=-1)
        # cv2.circle(frame, (self.coronaNaranja.I_x, self.coronaNaranja.I_y), 4, (100, 100, 0), thickness=-1)
        # cv2.circle(frame, (int(x), int(y)), 8, (255, 0, 0), thickness=2)
        # cv2.imshow("circulo_corona", frame)

        # de posiciones obtenidas vision...
        return (x, y, z, head), status


class Circulo:
    def __init__(self, lower, upper, trys, debug=False):
        self.upper_limit = upper
        self.lower_limit = lower
        self.x = 0
        self.y = 0
        self.I_x = 0
        self.I_y = 0
        self.last_located = 0
        self.located = False
        self.max_trys_location = trys

        # Bitwise-AND mask and original image
        self.kernel = np.ones((4, 4), np.uint8)

        # ^ v --slow
        self.K_x = kf.KalmanFilter(10, 1)
        self.K_y = kf.KalmanFilter(10, 1)
        # self.K_radius = kf.KalmanFilter(0.2, 1)

        self.roi_user = None

        self.debug = debug
        # Blob params
        # Filter by Circularity
        params = cv2.SimpleBlobDetector_Params()
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        # Set up the detector with default parameters.
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.panels = HSVRangePanels("circulo", self.lower_limit, self.upper_limit)

    # define a clamping function
    def clamp(self, n, minimum, maximum):
        return max(min(maximum, n), minimum)

    def set_lower(self, lower):
        self.lower_limit = lower
    def set_upper(self, upper):
        self.upper_limit = upper
    def set_range(self, lower, upper):
        self.lower_limit = lower
        self.upper_limit = upper

    def calculate_hsv_range(self, color, offset):
        lower = color.astype('int16') - offset
        upper = color.astype('int16') + offset
        np.clip(lower, 0, 255, out=lower)
        np.clip(upper, 0, 255, out=upper)
        lower[2] = 50
        upper[2] = 255
        print("L" + str(lower))
        print("U" + str(upper))
        self.set_range(lower, upper)

    def find(self, original, roi_user=None):

        frame = original.copy() # TODO Mucho % de mejora si hay que ganar fps. No copiar el frame.
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ## 2018 octubre. Ajusta range if modified en sliders panel
        if self.panels.get_flag_cambios_pendientes():
            self.panels.desactiva_flag_cambios_pendientes()
            self.set_range(self.panels.get_current_min_values(), self.panels.get_current_max_values())
        ##   ####   ####   ####   ####   ####   ####   ####   ####   ##

        mask = cv2.inRange(frame, self.lower_limit, self.upper_limit)
        # cv2.imshow("maskk", mask)
        frame_shape = frame.shape
        last_x_min = self.clamp(self.I_y - 30, 0, frame_shape[0]-1)
        last_x_max = self.clamp(self.I_y + 30, 0, frame_shape[0]-1)
        last_y_min = self.clamp(self.I_x - 45, 0, frame_shape[1]-1)
        last_y_max = self.clamp(self.I_x + 45, 0, frame_shape[1]-1)
        mask_roi = np.zeros(frame.shape[:2], np.uint8)
        mask_roi[last_x_min:last_x_max, last_y_min:last_y_max] = 255


        img_erosion = cv2.erode(mask, self.kernel, iterations=1)
        img_dilation = cv2.dilate(img_erosion, self.kernel, iterations=1)
        img_dilation = cv2.dilate(img_dilation, self.kernel, iterations=1)
        if self.debug: cv2.imshow("circulo_dilation", img_dilation)

        # cv2.imshow("circulo", mask)

        # res_dil = cv2.bitwise_and(frame, frame, mask=img_dilation)
        # cv2.imshow('target', res_dil)
        # res_ero_dil = cv2.bitwise_and(frame, frame, mask=img_ero_dil)

        # mask_inv = cv2.bitwise_not(img_dilation)
        # cv2.imshow('invertida', mask_inv)

        # Bitwise not (blob detection: Negro sobre blanco)
        mask_inv = cv2.bitwise_not(img_dilation)
        # cv2.imshow("punto", mask_inv)

        # cv2.imshow("maskara", mask_inv)
        # cv2.imshow("roi", mask_roi)
        # print(type(mask_inv), np.shape(mask_inv), type(mask_roi), np.shape(mask_roi))
        # print(mask_inv)

        flag_localizado = False
        if isinstance(roi_user, np.ndarray):
            # Detect blobs with mask
            focalizada = cv2.bitwise_and(mask_inv, roi_user)
            if self.debug: cv2.imshow("circulo focalizada externa", focalizada)
            keypoints = self.detector.detect(focalizada)
            # keypoints = self.detector.detect(mask_inv, mask=mask_roi)

            if keypoints:
                flag_localizado = True


        elif gb.solo_buscar_en_cercanias == False:
            if not flag_localizado and self.located:
                # Detect blobs with mask
                focalizada = cv2.bitwise_and(mask_inv, mask_roi)
                keypoints = self.detector.detect(focalizada)
                # keypoints = self.detector.detect(mask_inv, mask=mask_roi)
                # print("LOCATED")
                if keypoints:
                    flag_localizado = True

            if not flag_localizado and not self.located:
                keypoints = self.detector.detect(mask_inv)

        else:
            keypoints = self.detector.detect(mask_inv)

            # print("NOT LOCATED PREVIO")

        x = self.x
        y = self.y

        keypoints = keypoints[0:5]
        if keypoints:
            self.located = True
            self.last_located = 0
            # self.fames_sin_ver_circulo = 0
            # x = int(keypoints[0].pt[0])
            # y = int(keypoints[0].pt[1])

            # x, y = self.punto_mas_cercano(keypoints)
            kps = [(math.sqrt((kp.pt[0]-x)**2 + (kp.pt[1]-y)**2), index) for index, kp in enumerate(keypoints)]
            index_kp_cercano = sorted(kps)[0][1]
            x = int(keypoints[index_kp_cercano].pt[0])
            y = int(keypoints[index_kp_cercano].pt[1])

        else:
            # if self.last_located >= self.max_trys_location:
            if self.last_located >= self.max_trys_location:
                self.located = False
            else:
                self.last_located += 1


        if gb.disable_all_kalmans:
            self.x = x
            self.y = y
            self.I_x = int(self.x)
            self.I_y = int(self.y)
        else:
            # print("TRAS CNTS: ", x, y)
            xp = self.K_x.predict_and_correct(x)
            yp = self.K_y.predict_and_correct(y)

            # print("TRAS KALMAN: ", xp, yp)
            self.x = xp
            self.y = yp
            self.I_x = int(self.x)
            self.I_y = int(self.y)


        # print("SELF I_XY: ", self.I_x, self.I_y)

        # print(x, y)
        if self.debug: print("Dentro de circle: ", self.x, self.y)
        if self.debug: print("Dentro de circle I_: ", self.I_x, self.I_y)
        cv2.circle(original, (self.I_x, self.I_y), 5, (255, 0, 255), -1)
        # cv2.imshow("keypointsCirculo", original)
        return(mask_inv)



class punto_contorno:
    def __init__(self, lower, upper, id_panel, angulo_real=0, trys=3, debug=False):
        """
        :param lower: límite mínimo (Espacio color HSV - tupla (int, int, int) en rango ([0,179], [0, 255], [0,255])
        :param upper: límite máximo. igual arriba.
        :param id_panel: string nombre para sliders y imshows de debug
        :param angulo_real: posición real de la marca en el dron (Ej: -90 para marca a la izquierda, 0 en el frente, 90 marca a la derecha)
        :param trys: intentos busqueda cercana (int)
        :param debug: True/False para imshows y prints
        """
        self.debug = debug
        self.upper_limit = upper
        self.lower_limit = lower
        self.x = 0
        self.y = 0
        self.I_x = 0
        self.I_y = 0
        self.last_located = 0
        self.located = False
        self.max_trys_location = trys
        self.angulo_real = angulo_real

        # Bitwise-AND mask and original image
        self.kernel = np.ones((2, 2), np.uint8)
        self.kernel_burro = np.ones((3, 3), np.uint8)

        # ^ v --slow
        self.K_x = kf.KalmanFilter(10, 1)
        self.K_y = kf.KalmanFilter(10, 1)
        # self.K_radius = kf.KalmanFilter(0.2, 1)

        self.roi_user = None

        self.detector = cv2.SimpleBlobDetector_create()
        self.id_panel = id_panel
        self.panels = HSVRangePanels(self.id_panel, self.lower_limit, self.upper_limit)

    # define a clamping function
    def clamp(self, n, minimum, maximum):
        return max(min(maximum, n), minimum)

    def set_lower(self, lower):
        self.lower_limit = lower
    def set_upper(self, upper):
        self.upper_limit = upper
    def set_range(self, lower, upper):
        self.lower_limit = lower
        self.upper_limit = upper

    def calculate_hsv_range(self, color, offset):
        lower = color.astype('int16') - offset
        upper = color.astype('int16') + offset
        np.clip(lower, 0, 255, out=lower)
        np.clip(upper, 0, 255, out=upper)
        lower[2] = 50
        upper[2] = 255
        print("L" + str(lower))
        print("U" + str(upper))
        self.set_range(lower, upper)

    def find(self, original, roi_user=None):
        # ----------------------------
        frame = original.copy()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ## 2018 octubre. Ajusta range if modified en sliders panel
        if self.panels.get_flag_cambios_pendientes():
            self.panels.desactiva_flag_cambios_pendientes()
            self.set_range(self.panels.get_current_min_values(), self.panels.get_current_max_values())
            ##   ####   ####   ####   ####   ####   ####   ####   ####   ##


        if self.lower_limit[0] <= self.upper_limit[0]:
            mask = cv2.inRange(cv2.bitwise_and(frame,frame), self.lower_limit, self.upper_limit)
        else: # np.array([7, 255, 255])
            lower1 = self.lower_limit.copy()
            lower1[0] = 0
            upper1 = self.upper_limit.copy()
            lower2 = self.lower_limit.copy()
            upper2 = self.upper_limit.copy()
            upper2[0] = 179
            mask1 = cv2.inRange(cv2.bitwise_and(frame,frame), lower1, upper1)
            mask2 = cv2.inRange(cv2.bitwise_and(frame,frame), lower2, upper2)
            # if self.debug: cv2.imshow("mascara1", mask1)
            # if self.debug: cv2.imshow("mascara2", mask2)
            mask = cv2.bitwise_or(mask1, mask2)
        # mask = cv2.inRange(frame, self.lower_limit, self.upper_limit)

        frame_shape = frame.shape
        last_x_min = self.clamp(self.I_y - 30, 0, frame_shape[0] - 1)
        last_x_max = self.clamp(self.I_y + 30, 0, frame_shape[0] - 1)
        last_y_min = self.clamp(self.I_x - 45, 0, frame_shape[1] - 1)
        last_y_max = self.clamp(self.I_x + 45, 0, frame_shape[1] - 1)
        mask_roi = np.zeros(frame.shape[:2], np.uint8)
        mask_roi[last_x_min:last_x_max, last_y_min:last_y_max] = 255

        img_erosion = cv2.erode(mask, self.kernel, iterations=1)
        img_dilation = cv2.dilate(img_erosion, self.kernel, iterations=3)
        img_erosion = cv2.erode(img_dilation, self.kernel, iterations=1)

        mask_inv = img_erosion.copy()


        if self.debug: cv2.imshow(self.id_panel + " TRAZA1 - filtrado", mask_inv)

        if isinstance(roi_user, np.ndarray):
            if self.located:
                # Detect blobs with mask
                focalizada = cv2.bitwise_and(mask_inv, mask_roi)
                focalizada = cv2.bitwise_and(focalizada, roi_user)
            else:
                focalizada = cv2.bitwise_and(mask_inv, roi_user)
            focalizada = cv2.dilate(focalizada, self.kernel_burro, iterations=1)
            _, cnts, hier = cv2.findContours(focalizada, 1, cv2.CHAIN_APPROX_SIMPLE)
            if self.debug: cv2.imshow(self.id_panel + " TRAZA4 - Juntas", focalizada)
        else:
            cnts = []

        x = self.x
        y = self.y

        if cnts:
            # o_cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
            # if len(o_cnts) >= 2:
            #     print(" PRIMERO: ", cv2.contourArea(o_cnts[0]), " - SEGUNDO: ", cv2.contourArea(o_cnts[1]))
            cnt = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(cnt):
                self.located = True
                self.last_located = 0
                self.area_localizacion = mask_roi
            else:
                self.area_localizacion = None

            # print(cv2.contourArea(cnt))
            (x, y), radius = cv2.minEnclosingCircle(cnt)
        else:
            # if self.last_located >= self.max_trys_location:
            if self.last_located >= self.max_trys_location:
                self.located = False
            else:
                self.last_located += 1

        if gb.disable_all_kalmans:
            self.x = x
            self.y = y
            self.I_x = int(self.x)
            self.I_y = int(self.y)
        else:
            xp = self.K_x.predict_and_correct(x)
            yp = self.K_y.predict_and_correct(y)
            self.x = xp
            self.y = yp
            self.I_x = int(self.x)
            self.I_y = int(self.y)

        cv2.circle(original, (self.I_x, self.I_y), 3, rfs.tupla_BGR(self.id_panel.split()[0]), -1)

        # ---------------------------


class Corona:
    def __init__(self, lower, upper, trys, tamano_real, debug=False):

        self.debug = debug
        self.tamano_real_cm = tamano_real
        self.upper_limit = upper
        self.lower_limit = lower
        self.x = 0
        self.y = 0
        self.I_x = 0
        self.I_y = 0

        self.contador_inicializacion_vision = 0

        self.last_located = 0
        self.located = False
        self.max_trys_location = trys

        self.area_localizacion = None
        self.mask_zona = None

        self.radius = 0
        self.I_radius = 0

        # Bitwise-AND mask and original image
        self.kernel = np.ones((3, 3), np.uint8)
        # self.kernel_gordo = np.ones((10, 10), np.uint8)
        # self.kernel_redondo = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        # self.kernel_redondo_gordo = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))


        # v,^ --slow
        self.K_x = kf.KalmanFilter(6, 4)
        self.K_y = kf.KalmanFilter(5, 5)
        # self.K_radius = kf.KalmanFilter(0.75, 3) # <--- 0.15 antes (Corona Naranja)
        # self.K_radius = kf.KalmanFilter(0.15, 3)  # <--- (Aceptable :))
        self.K_radius = kf.KalmanFilter(0.05, 3)  # <--- 0.15 Mejor resultado.

    ##### JULIO 2018
        # self.K_x = kf.KalmanFilter(0.1, 0.2)
        # self.K_y = kf.KalmanFilter(0.1, 0.2)
        # self.K_radius = kf.KalmanFilter(0.1, 0.2)

        # # Blob params
        # # Filter by Circularity
        # params = cv2.SimpleBlobDetector_Params()
        # params.filterByCircularity = True
        # params.minCircularity = 0.1
        #
        # # Filter by Convexity
        # params.filterByConvexity = True
        # params.minConvexity = 0.87
        #
        # # Filter by Inertia
        # params.filterByInertia = True
        # params.minInertiaRatio = 0.01

        # Set up the detector with default parameters.
        # self.detector = cv2.SimpleBlobDetector_create(params)
        self.detector = cv2.SimpleBlobDetector_create()

        self.panels = HSVRangePanels("corona", self.lower_limit, self.upper_limit)


    # define a clamping function
    def clamp(self, n, minimum, maximum):
        return max(min(maximum, n), minimum)

    def set_lower(self, lower):
        self.lower_limit = lower
    def set_upper(self, upper):
        self.upper_limit = upper
    def set_range(self, lower, upper):
        self.lower_limit = lower
        self.upper_limit = upper

    def calculate_hsv_range(self, color, offset):
        lower = color.astype('int16') - offset
        upper = color.astype('int16') + offset
        np.clip(lower, 0, 255, out=lower)
        np.clip(upper, 0, 255, out=upper)
        lower[2] = 50
        upper[2] = 255
        print("L" + str(lower))
        print("U" + str(upper))
        self.set_range(lower, upper)


    def guarda_mask_corona(self, frame_shape):
        x_min = self.clamp(self.I_y - (self.I_radius + 15), 0, frame_shape[0] - 1)
        x_max = self.clamp(self.I_y + (self.I_radius + 15), 0, frame_shape[0] - 1)
        y_min = self.clamp(self.I_x - (self.I_radius + 15), 0, frame_shape[1] - 1)
        y_max = self.clamp(self.I_x + (self.I_radius + 15), 0, frame_shape[1] - 1)
        mask_zona = np.zeros(frame_shape[:2], np.uint8)
        mask_zona[x_min:x_max, y_min:y_max] = 255
        self.mask_zona = mask_zona

    def borra_mask_corona(self):
        self.mask_zona = None

    def find(self, original):

        frame = original.copy()
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # # Enero - 2019: equalizar no vale para localizar por color.
        # frame[:, :, 0] = cv2.equalizeHist(frame[:,:,0])
        # cv2.imshow("blurred", blurred)
        # cv2.imshow("equalized", cv2.cvtColor(frame, cv2.COLOR_HSV2BGR))

        # chapuza Febrero 2019:
        # Se hace un circulo alrededor del dron en el

        if gb.first_frame is None and self.contador_inicializacion_vision >= 20:
            mascara_alrededores = np.zeros(frame.shape[:2], np.uint8)
            cv2.circle(mascara_alrededores, (self.I_x, self.I_y), self.I_radius + 14 + 25, 255, -1)
            cv2.circle(mascara_alrededores, (self.I_x, self.I_y), self.I_radius + 15, 0, -1)
            color_alrededores = cv2.mean(frame, mascara_alrededores)
            print("AVERAGE COLOR Inicialización:", color_alrededores)

            # reconstruccion = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # en gris de momento, todo: hacer a color esta historia.
            reconstruccion = frame.copy()
            cv2.circle(reconstruccion, (self.I_x, self.I_y), self.I_radius + 15, color_alrededores[:3], -1)

            # reconstruccion[mascara_centro>1] = color_alrededores[:3]
            # cv2.imshow("reconstruccion", reconstruccion)
            # gb.first_frame = cv2.cvtColor(reconstruccion, cv2.COLOR_BGR2GRAY)
            gb.first_frame = reconstruccion

            cv2.imshow("reconstruccion imagen", reconstruccion)
            cv2.imshow("mascara inicializacion", cv2.cvtColor(cv2.bitwise_and(frame, frame, mask=mascara_alrededores), cv2.COLOR_HSV2BGR))
            # cv2.imshow("imagen inicial construida", gb.first_frame)

        elif gb.first_frame is None:
            self.contador_inicializacion_vision += 1

        if gb.first_frame is None:
            mask_mov = np.zeros(frame.shape[:2], np.uint8)
            cv2.circle(mask_mov, cfg.posicion_estimada_inicial, cfg.radio_zona_estimada_inicial, (rfs.tupla_BGR("blanco")), -1)
                    # esta rfs.tupla_BGR en realidad esta cogiendo el primer valor de los 3 que se dan...
            cv2.imshow("area estimada inicial", cv2.bitwise_and(frame, frame, mask=mask_mov))

        else:   # Caso normal tras inicializacion

            # en escala grises usabamos....
            # diferencia = cv2.subtract(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), gb.first_frame)

            diferencia_absdiff = cv2.absdiff(frame, gb.first_frame) # Feb 2019
            diferencia = cv2.cvtColor(diferencia_absdiff, cv2.COLOR_BGR2GRAY)
            if self.debug: cv2.imshow("absdiff resultado (diff)", diferencia_absdiff)

            blurred_diff = cv2.medianBlur(cv2.medianBlur(diferencia, 11), 11)

            #############
            ### PRUEBAS SUAVIZADO  - COMENTAR
            #############

            # blurred_diff = cv2.GaussianBlur(diferencia, (5, 5), 0)    # cambiamos al medianblur
            # blurred_diff = cv2.fastNlMeansDenoising(diferencia) # método lento de cojones.

            # bil9757 = cv2.bilateralFilter(diferencia, 9, 75, 7)
            # bil9457 = cv2.bilateralFilter(diferencia, 9, 45, 7)
            # bil9257 = cv2.bilateralFilter(diferencia, 9, 25, 7)
            # bil9057 = cv2.bilateralFilter(diferencia, 9, 5, 7)
            # bil5754 = cv2.bilateralFilter(diferencia, 5, 75, 4)
            # bil5454 = cv2.bilateralFilter(diferencia, 5, 45, 4)
            # bil5254 = cv2.bilateralFilter(diferencia, 5, 25, 4)
            # bil5057 = cv2.bilateralFilter(diferencia, 5, 5, 4)

            # med5x1 = cv2.medianBlur(diferencia, 5)
            # med5x2 = cv2.medianBlur(med5x1, 5)
            # med5x3 = cv2.medianBlur(med5x2, 5)
            # med11 = cv2.medianBlur(diferencia, 11)
            # med11x2 = cv2.medianBlur(med11, 11)         # <--- nos quedamos este
            # med11x3 = cv2.medianBlur(med11x2, 11)

            # medC = cv2.medianBlur(diferencia, 9)
            # medC2 = cv2.medianBlur(med11, 7)
            # medC3 = cv2.medianBlur(med11x2, 5)

            # med21 = cv2.medianBlur(diferencia, 21)
            # med21x2 = cv2.medianBlur(med21, 21)
            # med21x3 = cv2.medianBlur(med21x2, 21)
            # med31 = cv2.medianBlur(diferencia, 31)
            # med31x2 = cv2.medianBlur(med31, 31)
            # med31x3 = cv2.medianBlur(med31x2, 31)

            #median
            # cv2.imshow("median52", med5x2)
            # cv2.imshow("median53", med5x3)  # POCO
            # cv2.imshow("median112", med11x2)    # Puede valer - Nos quedamos este
            # cv2.imshow("median113", med11x3)    # Puede valer
            # cv2.imshow("median212", med21x2)    # Demasiado...
            # cv2.imshow("median213", med21x3)
            # cv2.imshow("median312", med31x2)
            # cv2.imshow("median313", med31x3)
            # cv2.imshow("medianC", medC)
            # cv2.imshow("medianC2", medC2)
            # cv2.imshow("medianC3", medC3)

            # gauss50 = cv2.GaussianBlur(diferencia, (5, 5), 0)
            # gauss510 = cv2.GaussianBlur(diferencia, (5, 5), 10)
            # gauss520 = cv2.GaussianBlur(diferencia, (5, 5), 20)
            # gauss110 = cv2.GaussianBlur(diferencia, (11, 11), 0)
            # gauss1110 = cv2.GaussianBlur(diferencia, (11, 11), 10)
            # gauss1120 = cv2.GaussianBlur(diferencia, (11, 11), 20)

            # #gauss
            # cv2.imshow("gauss50", gauss50)
            # cv2.imshow("gauss510", gauss510)
            # cv2.imshow("gauss520", gauss520)
            # cv2.imshow("gauss110", gauss110)
            # cv2.imshow("gauss1110", gauss1110)
            # cv2.imshow("gauss1120", gauss1120)

            # #bilateral
            # cv2.imshow("bil9757", bil9757)
            # cv2.imshow("bil9457", bil9457)
            # cv2.imshow("bil9257", bil9257)
            # cv2.imshow("bil9057", bil9057)
            # cv2.imshow("bil5754", bil5754)
            # cv2.imshow("bil5454", bil5454)
            # cv2.imshow("bil5254", bil5254)
            # cv2.imshow("bil5057", bil5057)

            # #### Pruebas UMBRAL
            # ret, mask_mov15 = cv2.threshold(blurred_diff, 15, 255, cv2.THRESH_BINARY)
            # ret, mask_mov30 = cv2.threshold(blurred_diff, 35, 255, cv2.THRESH_BINARY) # 30-40 parece OK
            # ret, mask_mov60 = cv2.threshold(blurred_diff, 60, 255, cv2.THRESH_BINARY)
            # ret, mask_mov90 = cv2.threshold(blurred_diff, 90, 255, cv2.THRESH_BINARY)
            #
            # cv2.imshow("mask_mov15", mask_mov15)
            # cv2.imshow("mask_mov30", mask_mov30)  #  <--
            # cv2.imshow("mask_mov60", mask_mov60)
            # cv2.imshow("mask_mov90", mask_mov90)


            ret, mask_mov = cv2.threshold(blurred_diff, 35, 255, cv2.THRESH_BINARY)
            # if self.debug: cv2.imshow("pre erodes (diff)", mask_mov)
            mask_mov = cv2.erode(mask_mov, kernel_circular(2), iterations=1)
            mask_mov = cv2.dilate(mask_mov, kernel_circular(4), iterations=1)
            mask_mov = cv2.erode(mask_mov, kernel_circular(5), iterations=1)
            mask_mov = cv2.dilate(mask_mov, kernel_circular(6), iterations=2)
            # if self.debug: cv2.imshow("substract (diff)", diferencia)
            # if self.debug: cv2.imshow("blurred (diff)", blurred_diff)
            if self.debug: cv2.imshow("after umbral (diff)", mask_mov)
            # frame = subs

        ## 2018 octubre. Ajusta range if modified en sliders panel
        if self.panels.get_flag_cambios_pendientes():
            self.panels.desactiva_flag_cambios_pendientes()
            self.set_range(self.panels.get_current_min_values(), self.panels.get_current_max_values())
            ##   ####   ####   ####   ####   ####   ####   ####   ####   ##

        if self.lower_limit[0] <= self.upper_limit[0]:
            mask = cv2.inRange(cv2.bitwise_and(frame,frame, mask=mask_mov), self.lower_limit, self.upper_limit)
        else: # np.array([7, 255, 255])
            lower1 = self.lower_limit.copy()
            lower1[0] = 0
            upper1 = self.upper_limit.copy()
            lower2 = self.lower_limit.copy()
            upper2 = self.upper_limit.copy()
            upper2[0] = 179
            mask1 = cv2.inRange(cv2.bitwise_and(frame,frame, mask=mask_mov), lower1, upper1)
            mask2 = cv2.inRange(cv2.bitwise_and(frame,frame, mask=mask_mov), lower2, upper2)
            if self.debug: cv2.imshow("mascara1", mask1)
            if self.debug: cv2.imshow("mascara2", mask2)
            mask = cv2.bitwise_or(mask1, mask2)

        frame_shape = frame.shape
        last_x_min = self.clamp(self.I_y - 80, 0, frame_shape[0]-1)
        last_x_max = self.clamp(self.I_y + 80, 0, frame_shape[0]-1)
        last_y_min = self.clamp(self.I_x - 120, 0, frame_shape[1]-1)
        last_y_max = self.clamp(self.I_x + 120, 0, frame_shape[1]-1)
        mask_roi = np.zeros(frame.shape[:2], np.uint8)
        mask_roi[last_x_min:last_x_max, last_y_min:last_y_max] = 255
        # cv2.imshow("ROI NUEVO MASKARA", mask_roi)

        # mask = cv2.bitwise_not(mask)
        if self.debug: cv2.imshow("mascara", mask)

        img_erosion = cv2.erode(mask, kernel_circular(2), iterations=2)
        img_dilation = cv2.dilate(img_erosion, kernel_circular(3), iterations=1)

        img_erosion = cv2.erode(img_dilation, kernel_circular(2), iterations=2)
        img_dilation = cv2.dilate(img_erosion, kernel_circular(5), iterations=1)

        img_erosion = cv2.erode(img_dilation, kernel_circular(2), iterations=2)
        img_dilation = cv2.dilate(img_erosion, kernel_circular(20), iterations=2)
        img_erosion = cv2.erode(img_dilation, kernel_circular(20), iterations=2)

        ########################################################################################################
        # <--- DRONE SIN CIRCULO
        # img_erosion = cv2.dilate(img_erosion, kernel_circular(10), iterations=2)        # <--- DRONE SIN CIRCULO
        # <--- DRONE SIN CIRCULO
        ########################################################################################################

        # img_erosion = cv2.erode(img_dilation, kernel_circular(10), iterations=2)
        # img_dilation = cv2.dilate(img_erosion, kernel_circular(4), iterations=1)
        #
        # img_dilation = cv2.erode(img_dilation, kernel_circular(3), iterations=2)

        # img_erosion = cv2.erode(mask, self.kernel, iterations=1)
        # img_dilation = cv2.dilate(img_erosion, self.kernel, iterations=1)
        #
        # img_erosion = cv2.erode(img_dilation, self.kernel, iterations=2)
        # img_dilation = cv2.dilate(img_erosion, self.kernel, iterations=2)

        # Bitwise not (blob detection: Negro sobre blanco)

        # cv2.imshow('maskara-loc', img_erosion)

        mask_inv = img_erosion.copy()

        if self.debug: cv2.imshow("filtrado", mask_inv)


        ################## Annadido para localizar en zona cercana
        ################## Pendiente ajustar mask_roi a tamanno acorde a lo que buscamos/velocidad de la cosa.

        # cv2.imshow("roi", mask_roi)
        # cv2.imshow("img-filtrada", mask_inv)
        if self.located:
            # Detect blobs with mask
            focalizada = cv2.bitwise_and(mask_inv, mask_roi)
            # cv2.imshow("img-focalizada", mask_inv)
            _, cnts, hier = cv2.findContours(focalizada, 1, cv2.CHAIN_APPROX_SIMPLE)
            # keypoints = self.detector.detect(mask_inv, mask=mask_roi)

            if not cnts:
                _, cnts, hier = cv2.findContours(mask_inv, 1, cv2.CHAIN_APPROX_SIMPLE)
                # print("NOT LOCATED ACTUAL")
            else:
                cnt = max(cnts, key=cv2.contourArea)
                if cv2.contourArea(cnt) <= 100:   ####### Ojete con este valor metido a meno (tamano)
                    _, cnts, hier = cv2.findContours(mask_inv, 1, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, cnts, hier = cv2.findContours(mask_inv, 1, cv2.CHAIN_APPROX_SIMPLE)
            # print("NOT LOCATED PREVIO")

        x = self.x
        y = self.y
        radius = self.radius

        if cnts:
            # Feb 2019 - Filtrado solo contornos grandotes.
            cnts = [cnt for cnt in cnts if cv2.contourArea(cnt) > 600 and cv2.contourArea(cnt) < 2100 and rfs.circularidad(cnt) <= 1.4]
        if cnts:
            # Feb 2019 - Me quedo el de forma más circular de los grandotes
            cnt = min(cnts, key=rfs.circularidad)

            # o_cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
            # if len(o_cnts) >= 2:
            #     print(" PRIMERO: ", cv2.contourArea(o_cnts[0]), " - SEGUNDO: ", cv2.contourArea(o_cnts[1]))
            # cnt = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(cnt):
                self.located = True
                self.area_localizacion = mask_roi
                self.guarda_mask_corona(frame_shape)
            else:
                self.area_localizacion = None
                self.borra_mask_corona()


            # print(cv2.contourArea(cnt))
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
        else:
            # if self.last_located >= self.max_trys_location:
            if self.last_located >= self.max_trys_location:
                self.located = False
                self.borra_mask_corona()
            else:
                self.last_located += 1

            ################## Fin mods, zona cercana

        if gb.disable_all_kalmans:
            self.x = x
            self.y = y
            self.I_x = int(self.x)

            self.I_y = int(self.y)
        else:
            xp = self.K_x.predict_and_correct(x)
            yp = self.K_y.predict_and_correct(y)
            self.x = xp
            self.y = yp
            self.I_x = int(self.x)
            self.I_y = int(self.y)

        # kalman radio
        radio = self.K_radius.predict_and_correct(radius)
        self.radius = radio
        self.I_radius = int(self.radius)

        # print("SELF I_XY: ", self.I_x, self.I_y)

        # print(x, y)
        cv2.circle(original, (self.I_x, self.I_y), self.I_radius, (255, 255, 255), 1)
        # cv2.imshow("keypointsCORONA", original)


        #### Pruebas 2018 Julio
        # im_with_keypoints = cv2.drawKeypoints(frame, bk_keypoints, np.array([]), (0, 0, 255),
        #                                       cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #cv2.imshow("keypointsCORONA", im_with_keypoints)

        try:
            pass
            # print(keypoints[0].pt, keypoints[0].size, keypoints[0].octave)
        except:
            pass


        if self.located:
            mask_retornada = np.zeros(mask_inv.shape, np.uint8)
            # Draw a diagonal blue line with thickness of 5 px
            try:
                cv2.circle(mask_retornada, center, radius+6, 255, -1)
            except UnboundLocalError:
                # mask_retornada = cv2.dilate(mask_inv, kernel_circular(5), iterations=1)
                mask_retornada = mask_roi
        else:   #no localizada
            mask_retornada = cv2.dilate(mask_inv, kernel_circular(10), iterations=1)
        if self.debug: cv2.imshow("mascara retorada", mask_retornada)
        return(mask_retornada)


class HSVRangePanels():
    """
        saca sliders control valores HSV min y max.
        window_name: nombre ventana para los sliders.
    """
    def __init__(self, window_name, initial_min, initial_max):
        self.window = window_name
        self.cambios_pendientes_flag = False

        #################### TODO tomar de fichero valor inicial - Automatico?
        self.h_min, self.s_min, self.v_min = initial_min
        self.h_max, self.s_max, self.v_max = initial_max
        ####################

        self.create_trackbars()

    def create_trackbars(self):
        cv2.namedWindow(self.window)
        cv2.resizeWindow(self.window, 200, 300)
        # cv2.moveWindow('target', 100, 600)
        cv2.createTrackbar('Hue min', self.window, self.h_min, 180, self.activa_cambios_pendientes)
        cv2.createTrackbar('Hue max', self.window, self.h_max, 180, self.activa_cambios_pendientes)
        cv2.createTrackbar('Sat min', self.window, self.s_min, 255, self.activa_cambios_pendientes)
        cv2.createTrackbar('Sat max', self.window, self.s_max, 255, self.activa_cambios_pendientes)
        cv2.createTrackbar('Val min', self.window, self.v_min, 255, self.activa_cambios_pendientes)
        cv2.createTrackbar('Val max', self.window, self.v_max, 255, self.activa_cambios_pendientes)

    # TODO metodo oculta/muestra ventanas sliders

    @staticmethod
    def refresh_sliders_from_globals():
        pass

    def activa_cambios_pendientes(self, value):
        self.cambios_pendientes_flag = True

        # TODO atacar solo el cambiante.
        self.h_min = cv2.getTrackbarPos('Hue min', self.window)
        self.h_max = cv2.getTrackbarPos('Hue max', self.window)
        self.s_min = cv2.getTrackbarPos('Sat min', self.window)
        self.s_max = cv2.getTrackbarPos('Sat max', self.window)
        self.v_min = cv2.getTrackbarPos('Val min', self.window)
        self.v_max = cv2.getTrackbarPos('Val max', self.window)

    def desactiva_flag_cambios_pendientes(self):
        self.cambios_pendientes_flag = False

    def get_flag_cambios_pendientes(self):
        return(self.cambios_pendientes_flag)

    def get_current_min_values(self):
        """
        :return: numpy array with [H,S,V] values (min)
        """
        return(np.array([self.h_min, self.s_min, self.v_min]))

    def get_current_max_values(self):
        """
        :return: numpy array with [H,S,V] values (MAX)
        """
        return(np.array([self.h_max, self.s_max, self.v_max]))

