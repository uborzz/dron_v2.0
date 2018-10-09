# -*- coding: utf-8 -*-

### Buffer frames de un video en cola en un hilo para tratamiento en otro hilo
# A medias. - Lo paso a
#
# - mods by rady

from threading import Thread
import cv2
from queue import Queue

class FileVideoStream:

    def __init__(self, path, res_w, res_h, fps, queueSize=128):
        self.stream = cv2.VideoCapture(path)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
        self.stream.set(cv2.CAP_PROP_FPS, fps)
        self.stopped = False

        self.Q = Queue(maxsize=queueSize)

    def start(self):
        # thread to read frames from the file video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely
        while True:
            # if the thread indicator variable is set, stop the
            # thread
            if self.stopped:
                return

            # otherwise, ensure the queue has room in it
            if not self.Q.full():
                # read the next frame from the file


                # if the `grabbed` boolean is `False`, then we have
                # reached the end of the video file
                if not grabbed:
                    self.stop()
                    return

                # add the frame to the queue
                self.Q.put(frame)

    def read(self):
        # return next frame in the queue
        return self.Q.get()

    def more(self):
        # return True if there are still frames in the queue
        return self.Q.qsize() > 0

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True