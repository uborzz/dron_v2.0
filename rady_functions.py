import numpy as np
import cv2
import globals as gb

def get_undistort_map():
    try:
        raise ("DESACTIVADO") # trick
        # Con la correccion ralentizamos un 15-20% másaswa
        # Aprox. observado: 20 fps OK con corrección de la distorsiónq vs 24 fps OK sin corrección - en 640x480

        # load the camera calibration parameters
        calfile = np.load('camera_calibration/calibration.npz')
        newcameramtx = calfile['newcameramtx']
        mtx = calfile['mtx']
        dist = calfile['dist']
        roi = calfile['roi']

        map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (width, height), cv2.CV_32FC1)
        undistort = True
        print("Camera calibration params loaded")

    except:
        map1, map2 = None, None
        undistort = False
        print("WARNING: Going without camera calibration")

    return undistort, map1, map2


class RingBuffer():
    "A 1D ring buffer using numpy arrays"
    def __init__(self, length):
        self.length = length
        self.data = np.zeros(length, dtype='f')
        self.index = 0

    def extend(self, x):
        "adds array x to ring buffer"
        x_index = (self.index + np.arange(1)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

    def get(self):
        "Returns the first-in-first-out data in the ring buffer"
        idx = (self.index + np.arange(self.data.size)) % self.data.size
        return self.data[idx]

    def mean(self):
        return (self.data.mean())


class ConfigPanels():

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

    def nothing(self, value):
        gb.refresca_params_flag = True

    def nothing_target(self):
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
