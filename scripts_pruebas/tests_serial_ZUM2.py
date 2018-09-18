import serial
import time
import cv2

cam = cv2.VideoCapture(1)

throttle_middle = 1600  # up (+) and down (-)
aileron_middle = 1500  # left (-) and right (+)
elevator_middle = 1500  # forward (+) and backward (-)
rudder_middle = 1500  # yaw left (-) and yaw right (+)

# define engines off PPM value for throttle
throttle_off = 1000

usb_port = 'COM6' # Windows
arduino = serial.Serial(usb_port, 115200, timeout=.01)

# wait a bit for the connection to settle
time.sleep(2)

# tell the drone to calibrate the gyroscope
# left stick to bottom left
command = "%i,%i,%i,%i" % (throttle_off, 1000, 1000, rudder_middle)
command = command + "\n"

print("Starting calibration.")
arduino.write(command.encode())
# after 1.5 second, left stick goes back to neutral position
time.sleep(1.5)
command = "%i,%i,%i,%i" % (1600, aileron_middle, elevator_middle, rudder_middle)
command = command + "\n"
arduino.write(command.encode())
time.sleep(1)

thr, ail, ele, rud = 1600, aileron_middle, elevator_middle, rudder_middle

counter=0
paro = True

while True:


    counter+=1
    # print(counter)

    _, frame = cam.read()
    cv2.imshow('image', frame)

    k = cv2.waitKey(1)

    if k == ord("q"):
        thr, ail, ele, rud = 0, 0, 0, 0
        paro = True

    elif k == ord("w"):
        ele -= 100

    elif k == ord("d"):
        ail -= 100

    elif k == ord("s"):
        ele += 100

    elif k == ord("a"):
        ail += 100

    if k == ord("o"):
        thr += 100

    elif k == ord("l"):
        thr -= 100

    if k == ord("e"):
        paro = False
        thr, ail, ele, rud = 2000, aileron_middle, elevator_middle, rudder_middle

    if k == 27:
        thr, ail, ele, rud = 0, 0, 0, 0
        command = "%i,%i,%i,%i" % (thr, ail, ele, rud)
        command = command + "\n"
        arduino.write(command.encode())
        print("FINISHING")
        break

    command = "%i,%i,%i,%i" % (thr, ail, ele, rud)
    command = command + "\n"
    arduino.write(command.encode())

