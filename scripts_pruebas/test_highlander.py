from threading import Thread

import rady_functions as rfs
from scripts_pruebas import test_highlander2 as ttt

a = rfs.Recorder()
a.metealgo()
b = rfs.Recorder()
b.metealgo()
print("a tras ins a y b inicial", a.muestra_xRecord())

print(a is b)

t = Thread(target=ttt.main(), args=())
t.daemon = True
t.start()

# sleep(10)
print("a", str(a))
print("b", str(b))
print(a.muestra_xRecord())

