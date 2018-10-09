
import sys
import time

import cv2
import numpy as np
from imutils.video import FPS
from versiones_backups import PID

import dron
from old_and_backups.filevideostreamEnh import FileVideoStream

# Version INFO
vpy = sys.version_info
print("Python Version: {}.{}.{}".format(vpy[0],vpy[1],vpy[2]))
print("OpenCV Version: " + cv2.__version__)
print("Numpy Version: " + np.__version__)

try:
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
    undistort = False
    print("WARNING: Going without camera calibration")


# Resolution
width = 640
height = 480
fps = 30

frame = None
hsv = None

def faster_webcam(path):
    # path a la direccion del video o numero entero para la camara

    # start the file video stream thread and allow the buffer to
    # start to fill
    print("[INFO] starting video file thread...")
    fvs = FileVideoStream(path, 24, 16, 20).start()
    time.sleep(1.0)

    # start the FPS timer
    fps = FPS().start()


    # while True:

    # loop over frames from the video file stream
    while True:
        # grab the frame from the threaded video file stream, resize
        # it, and convert it to grayscale (while still retaining 3
        # channels)
        frame = fvs.read()
        # frame = imutils.resize(frame, width=240)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = np.dstack([frame, frame, frame])

        # display the size of the queue on the frame
        cv2.putText(frame, "Queue Size: {}".format(fvs.Q.qsize()),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        print(frame.shape, frame.size)

        # show the frame and update the FPS counter
        cv2.imshow("image", frame)
        if cv2.waitKey(1) == ord('q'):
            break
        while not fvs.more():
            pass
        fps.update()

        # if cv2.waitKey(1) == ord('q'):
        #     break

    # stop the timer and display FPS information
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

    # do a bit of cleanup
    cv2.destroyAllWindows()
    fvs.stop()

def click(event,x,y,flags,param):

    global frame
    global hsv
    global lower_blue
    global upper_blue

    # if event == cv2.EVENT_LBUTTONDOWN:
    #     pass

    if event == cv2.EVENT_LBUTTONUP:
        print("Color picker!")
        color = hsv[y,x]
        print(color)


        lower_blue = color.astype('int16') - 30
        upper_blue = color.astype('int16') + 30
        np.clip(lower_blue, 0, 255, out=lower_blue)
        np.clip(upper_blue, 0, 255, out=upper_blue)
        lower_blue[2] = 0
        upper_blue[2] = 255
        print("L" + str(lower_blue))
        print("U" + str(upper_blue))

        # cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
        # cv2.imshow("image", i

def tracking():
    global frame
    global hsv

    # Resolution
    width = 640
    height = 480
    fps = 30
    mirror = True

    cam = cv2.VideoCapture(1)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cam.set(cv2.CAP_PROP_FPS, fps)
    # cam = cv2.VideoCapture(1)
    # cam.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
    # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
    # cam.set(cv2.CAP_PROP_FPS, fps)
    # cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    # cam.set(cv2.CAP_PROP_RECTIFICATION, 0)
    # cam.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 0)
    # cam.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 0)

    # Instancia el dron
    midron = dron.Dron("COM7")

    image_number = 1

    # Lectura frames per second
    start = time.time()
    buffer = RingBuffer(10)
    avg_fps = 0.0
    counter = 0

    # clicks
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', click)

    while True:
        _, frame = cam.read()

        # Espejo si/no?
        if mirror:
            frame = cv2.flip(frame, 1)

        # Camara calibrada? Obtenemos la transformacin
        if undistort:
            # cropea el frame unudistorted
            pass

        x, y, z, head =
        midron.set_position(x, y, z, head)

        # PosicionX0 = (v1.x + v2.x) / 2;
        # PosicionX0 = PosicionX0 - (Width / 2);
        # PosicionX0 = PosicionX0 / PixCm;
        # PosicionX0 = K_X.predict_and_correct(PosicionX0);
        #
        # PosicionY0 = (v1.y + v2.y) / 2;
        # PosicionY0 = - PosicionY0 + (Height / 2);
        # PosicionY0 = PosicionY0 / PixCm;
        # PosicionY0 = K_Y.predict_and_correct(PosicionY0);

        salida_x = midron.x_pid.output
        midron.x_pid.update()
        midron.y_pid.update()
        midron.z_pid.update()


        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        cv2.inRange
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        kernel = np.ones((4, 4), np.uint8)
        kernel_fino = np.ones((1, 1), np.uint8)
        img_erosion = cv2.erode(mask, kernel_fino, iterations=1)
        img_dilation = cv2.dilate(mask, kernel, iterations=1)
        img_ero_dil = cv2.dilate(img_erosion, kernel, iterations=1)

        res_ero = cv2.bitwise_and(frame, frame, mask=img_erosion)
        res_dil = cv2.bitwise_and(frame, frame, mask=img_dilation)
        res_ero_dil = cv2.bitwise_and(frame, frame, mask=img_ero_dil)

        # cv2.imshow('Erosion', res_ero)
        # cv2.imshow('Dilation', res_dil)

        cv2.imshow('Erosion + Dilation', img_ero_dil)

        kernelOpen = np.ones((2, 2), np.uint8)
        kernelClose = np.ones((10, 10), np.uint8)
        maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

        cv2.imshow("maskClose", maskClose)

        try:
            cv2.drawContours(frame, cnt[0], 0, (0, 255, 0), 3)
        except:
            pass
        cv2.imshow('image', frame)

        cv2.imshow('image', frame)
        # cv2.imshow('mask', mask)
        # cv2.imshow('res', res)

        # blur = cv2.GaussianBlur(frame, (5, 5), 0)
        # maskAfterBlur = cv2.inRange(hsv, lower_blue, upper_blue)

        # print(maskClose)
    #cnt = cv2.findContours(maskClose,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        # cnt = cv2.findContours(maskClose, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print(cnt[0])
        # try:
        #     imnn = cv2.cvtColor(maskAfterBlur)
        #     ret, thresh = cv2.threshold(imnn, 0, 1, 0)
        #     contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #     cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
        # except:
        #     cv2.imshow('mask after blur', maskAfterBlur)
        #     pass

        k = cv2.waitKey(1)

        if k == ord('q'):
            # cnt_mem = cnt
            print("q")
            pass
        elif k == ord("w"):
            print("w")
            pass
        elif k == ord("s"):
            # cv2.imwrite('./captures/bw{}.jpg'.format(image_number), cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            cv2.imwrite('./captures/image{}.jpg'.format(image_number), frame)
            image_number += 1
            print("salvando captura!")
        elif k == 27:
            break

        elif k == ord("z"):
            n = np.load("calibration_NOMIzA.npz")
            print(n)


        end = time.time()
        buffer.extend(end-start)
        start = time.time()
        if not counter % 30:
            print(buffer.mean(), 1/buffer.mean(), counter)
        counter+=1

    cv2.destroyAllWindows()


def show_webcam(res_w, res_h, fps, mirror=False):

    global frame
    global hsv

    global lower_blue
    global upper_blue
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])

    cam = cv2.VideoCapture(1)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
    cam.set(cv2.CAP_PROP_FPS, fps)
    # cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    # cam.set(cv2.CAP_PROP_RECTIFICATION, 0)
    # cam.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 0)
    #
    # cam.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 0)

    image_number = 1

    # Lectura frames per second
    start = time.time()
    buffer = RingBuffer(10)
    avg_fps = 0.0
    counter = 0

    # PID
    pid1 = PID.PID()
    pid2 = PID.PID()
    pid3 = PID.PID()
    feedback1 = 0
    feedback2 = 0
    feedback3 = 0

    # clicks
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', click)

    while True:
        _, frame = cam.read()

        if mirror:
            frame = cv2.flip(frame, 1)

        pid1.update(feedback1)
        output1 = pid1.output
        pid2.update(feedback2)
        output2 = pid2.output
        pid3.update(feedback3)
        output3 = pid3.output

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        cv2.inRange
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        kernel = np.ones((4, 4), np.uint8)
        kernel_fino = np.ones((1, 1), np.uint8)
        img_erosion = cv2.erode(mask, kernel_fino, iterations=1)
        img_dilation = cv2.dilate(mask, kernel, iterations=1)
        img_ero_dil = cv2.dilate(img_erosion, kernel, iterations=1)

        res_ero = cv2.bitwise_and(frame, frame, mask=img_erosion)
        res_dil = cv2.bitwise_and(frame, frame, mask=img_dilation)
        res_ero_dil = cv2.bitwise_and(frame, frame, mask=img_ero_dil)

        # cv2.imshow('Erosion', res_ero)
        # cv2.imshow('Dilation', res_dil)

        cv2.imshow('Erosion + Dilation', img_ero_dil)

        kernelOpen = np.ones((2, 2), np.uint8)
        kernelClose = np.ones((10, 10), np.uint8)
        maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

        cv2.imshow("maskClose", maskClose)

        try:
            cv2.drawContours(frame, cnt[0], 0, (0, 255, 0), 3)
        except:
            pass
        cv2.imshow('image', frame)

        cv2.imshow('image', frame)
        # cv2.imshow('mask', mask)
        # cv2.imshow('res', res)

        # blur = cv2.GaussianBlur(frame, (5, 5), 0)
        # maskAfterBlur = cv2.inRange(hsv, lower_blue, upper_blue)

        # print(maskClose)
    #cnt = cv2.findContours(maskClose,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        # cnt = cv2.findContours(maskClose, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print(cnt[0])
        # try:
        #     imnn = cv2.cvtColor(maskAfterBlur)
        #     ret, thresh = cv2.threshold(imnn, 0, 1, 0)
        #     contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #     cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
        # except:
        #     cv2.imshow('mask after blur', maskAfterBlur)
        #     pass

        k = cv2.waitKey(1)

        if k == ord('q'):
            # cnt_mem = cnt
            print("q")
            pass
        elif k == ord("w"):
            print("w")
            pass
        elif k == ord("s"):
            # cv2.imwrite('./captures/bw{}.jpg'.format(image_number), cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            cv2.imwrite('./captures/image{}.jpg'.format(image_number), frame)
            image_number += 1
            print("salvando captura!")
        elif k == 27:
            break

        elif k == ord("z"):
            n = np.load("calibration_NOMIzA.npz")
            print(n)


        end = time.time()
        buffer.extend(end-start)
        start = time.time()
        if not counter % 30:
            print(buffer.mean(), 1/buffer.mean(), counter)
        counter+=1

    cv2.destroyAllWindows()


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

def main():
    # show_webcam(640, 480, 30, mirror=True)
    # faster_webcam(1)
    tracking()

if __name__ == '__main__':
    main()