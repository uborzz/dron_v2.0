from threading import Thread
import cv2

"""
rady

basado en webcamvideostream de pyimagesearch para raspi camera.

Camera props:
    CAP_PROP_POS_MSEC Current position of the video file in milliseconds.
    CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
    CAP_PROP_POS_AVI_RATIO Relative position of the video file: 0 - start of the film, 1 - end of the film.
    CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
    CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
    CAP_PROP_FPS Frame rate.
    CAP_PROP_FOURCC 4-character code of codec.
    CAP_PROP_FRAME_COUNT Number of frames in the video file.
    CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
    CAP_PROP_MODE Backend-specific value indicating the current capture mode.
    CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
    CAP_PROP_CONTRAST Contrast of the image (only for cameras).
    CAP_PROP_SATURATION Saturation of the image (only for cameras).
    CAP_PROP_HUE Hue of the image (only for cameras).
    CAP_PROP_GAIN Gain of the image (only for cameras).
    CAP_PROP_EXPOSURE Exposure (only for cameras).
    CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
    CAP_PROP_WHITE_BALANCE Currently unsupported
    CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
"""

class Stream:
    def __init__(self, src=0, resolution=None, framerate=30):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FPS, framerate)
        if resolution:
            self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

        (self.grabbed, self.frame) = self.stream.read()
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False
        self.records = list()

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()


    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def menu_config(self):
        # muestra menu configuracion params de la camara
        self.stream.set(cv2.CAP_PROP_SETTINGS, 0)
