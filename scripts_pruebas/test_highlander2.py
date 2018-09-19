import rady_functions as rfs
import time

def main():
    a = rfs.Recorder()
    cnt = 0
    while True:
        cnt+=1
        time.sleep(2)
        a.metealgo()
        if cnt == 5: a = rfs.Recorder()
        a.muestra_xRecord()
        print("a hilo", str(a))