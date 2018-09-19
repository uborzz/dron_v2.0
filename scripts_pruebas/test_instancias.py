import time

import dron

# test_instancias2.funn()
dron1, dron2 = None, None
# dron1 = dron.take_dron()
# dron2 = dron.take_dron()
dron.create_dron()
print(dron1, dron2)
time.sleep(1)
from scripts_pruebas import test_instancias2

dron3, dron4 = test_instancias2.funn()
# from threading import Thread
# t = Thread(target=test_instancias2.funn, args=())
# t.daemon = True
# t.start()

print(dron1, dron2, dron3, dron4)