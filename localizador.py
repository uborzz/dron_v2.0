import sys
import cv2
import numpy as np
import time
import queue
import kalman_jc as kf
import math
import globals as gb

def kernel_cuadrado(n):
    return np.ones((n, n), np.uint8)

def kernel_circular(n):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))


class Localizador:
    # Tendra 2 bolas y una corona - Corona solo
    def __init__(self, distancia_camara_suelo, debug=False, info=False, kalman_angle=True):
        self.debug = debug
        self.info = info

        lower_rojo = np.array([175, 50, 50])
        upper_rojo = np.array([5, 255, 255])

        lower_AZUL = np.array([109, 50, 50])
        upper_AZUL = np.array([115, 255, 255])



        # LABO
        lower_corona = np.array([20, 50, 50])
        upper_corona = np.array([30, 255, 255])     ## <- Naranja ~ 20-25
        lower_verde = np.array([38, 50, 50])
        upper_verde = np.array([52, 255, 255])

        # CASA
        lower_corona = np.array([5, 90, 140])
        upper_corona = np.array([25, 200, 200])
        lower_verde = np.array([34, 125, 120])
        upper_verde = np.array([45, 255, 255])

        self.distancia_camara_suelo = distancia_camara_suelo
        self.coronaNaranja = Corona(lower_corona, upper_corona, 3, 12, debug=self.debug)    # <-- Corona naranja habitual. Espera reduccion peso.
        # self.coronaNaranja = Corona(lower_rojo, upper_rojo, 3, tamano_real=11, debug=self.debug)      # <-- Carcasa Dron Rojo prueba.
        self.circuloVerde = Circulo(lower_verde, upper_verde, 3, debug=self.debug)

        # self.K_x = kf.KalmanFilter(1, 1)
        # self.K_y = kf.KalmanFilter(1, 1)
        # self.K_head = kf.KalmanFilter(1, 1)
        # self.K_z = kf.KalmanFilter(1, 1)

        self.head = 0
        self.K_head = kf.KalmanFilter(5, 5)

        # self.coronaNaranja = Corona(2)
        pass


    # define a clamping function
    def clamp(self, n, minimum, maximum):
        return max(min(maximum, n), minimum)


    def estima_altura(self, vista_objeto, hmax=200, corona=13):
        # corona es el tamano real en cm de la corona que buscamos
        # Altura = 200 * (1 - (Do / objeto)) # H = 200
        # print("objeto: ", str(objeto))
        if vista_objeto:
            # aprox para 14 cm - 10945
            return(hmax - 10945*(corona/13)/vista_objeto) # Max distancia 200 cm.
        else:
            return 0



    def posicion_dron(self, frame):
        status = True
        # Aplicar filtro Kalman

        # resized_frame = cv2.resize(frame, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_CUBIC)
        # frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # self.circuloAmarillo.find(frameHSV) # encuentra punto amarillo
        self.coronaNaranja.find(frame)  # encuentra corona
        self.circuloVerde.find(frame, roi_user=self.coronaNaranja.mask_zona)  # encuentra punto verde

        x = self.coronaNaranja.I_x
        y = self.coronaNaranja.I_y


        # x = self.K_x.predict_and_correct(x)
        # y = self.K_x.predict_and_correct(y)
        # z = self.tamano_maximo_corona()

        # Ajuste Z
        z = self.estima_altura(self.coronaNaranja.radius * 2, hmax=self.distancia_camara_suelo, corona=self.coronaNaranja.tamano_real_cm) # referencia a radio del circulo de la corona

        if gb.solo_buscar_en_cercanias and self.circuloVerde.last_located >=1:
            head = self.head
            status = False
        else:
            head = math.atan2(x-self.circuloVerde.x, y-self.circuloVerde.y) # radianes
            head = int(math.degrees(head)) + 180

        if not gb.disable_all_kalmans:
            if head <= 10 or head >= 350:
                pass
            elif gb.kalman_angle:
                headp = self.K_head.predict_and_correct(head)
                head = int(headp)

        self.head = head

        # Para dibujar la flecha direccion
        x2 = int(x - self.coronaNaranja.radius * math.sin(math.radians(head-180)))
        y2 = int(y - self.coronaNaranja.radius * math.cos(math.radians(head-180)))

        if self.info: print("[LOCALIZADOR]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(x, y, z, head))

        cv2.arrowedLine(frame, (x,y), (x2, y2), (50,255,230), 2)



        # DRAW:
        # cv2.circle(frame, (self.circuloVerde.I_x, self.circuloVerde.I_y), 4, (0,255,0), thickness=-1)
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

        frame = original.copy()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

        keypoints = keypoints[0:1]
        if keypoints:
            self.located = True
            self.last_located = 0
            self.fames_sin_ver_circulo = 0
            x = int(keypoints[0].pt[0])
            y = int(keypoints[0].pt[1])
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
        cv2.circle(original, (self.I_x, self.I_y), 5, (255, 0, 255), -1)
        # cv2.imshow("keypointsCirculo", original)



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
        self.K_radius = kf.KalmanFilter(0.75, 3) # <--- 0.15 antes (Corona Naranja)

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
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # cv2.imshow("hsv", frame)

        if self.lower_limit[0] <= self.upper_limit[0]:
            mask = cv2.inRange(frame, self.lower_limit, self.upper_limit)
        else: # np.array([7, 255, 255])
            lower1 = self.lower_limit.copy()
            lower1[0] = 0
            upper1 = self.upper_limit.copy()
            lower2 = self.lower_limit.copy()
            upper2 = self.upper_limit.copy()
            upper2[0] = 180
            mask1 = cv2.inRange(frame, lower1, upper1)
            mask2 = cv2.inRange(frame, lower2, upper2)
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
            # o_cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
            # if len(o_cnts) >= 2:
            #     print(" PRIMERO: ", cv2.contourArea(o_cnts[0]), " - SEGUNDO: ", cv2.contourArea(o_cnts[1]))
            cnt = max(cnts, key=cv2.contourArea)
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
        cv2.circle(original, (self.I_x, self.I_y), self.I_radius, (255, 255, 255), 2)
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