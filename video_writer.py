import cv2
from datetime import datetime

class video_writer:

    __instance = None

    def __new__(cls):
        if cls.__instance == None:
            cls.__instance = object.__new__(cls)
            cls.__instance.rady_init()
        # print("Devolviendo instancia:", str(cls.__instance))
        # print("Valor de OUT actual:", str(cls.__instance.out))
        return cls.__instance

    def rady_init(self):
        self.out = None
        self.filename = None
        self.enabled = False
        # print("ENTRANDO EN RADY INIT...")

    def is_initialized(self):
        if self.filename:
            return True
        return False

    def initialize(self, filename=None, dims=(320,240)):
        # mejorar con mas params en futuro, ahora mismo vale asi.

        if filename == None:
            # automatic nombre
            name = "{:%d_%H_%M}.avi".format(datetime.now())
            self.filename = 'videos/' + name

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.filename, fourcc, 20.0, dims)

    def write(self, frame):
        # TODO frame no igual tamano video? -> escalar.

        if self.out == None:
            raise Exception("Fichero de escritura no especificado. Llama a initialize primero.")
        try:
            self.out.write(frame)
        except:
            raise

    def turn_on(self):
        self.enabled = True

    def turn_off(self):
        self.enabled = False

    def is_enabled(self):
        return self.enabled

    def get_filename(self):
        return self.filename

    def release(self):
        self.out.release()