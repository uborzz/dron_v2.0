import numpy as np

def get_undistort_map():
    try:
        raise ("DESACTIVADO") # trick
        # Con la correccion ralentizamos un 15-20% másaswa
        # Aprox. observado: 20 fps OK con corrección de la distorsiónq vs 24 fps OK sin corrección - en 640x480

        # load the camera calibration parameters
        calfile = np.load('calibration.npz')
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