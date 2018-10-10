# import dron
from time import sleep
import cv2
import cv2.aruco as aruco

# print(cv2.__version__)
#
# midron = dron.Dron("COM7")
# midron.writeln("######################")
# while True:
#     print("HOLA")
#     midron.writeln("HOLA")
#     sleep(1)

# Standard imports
import cv2
import numpy as np
#
# # Read image
#
#
# # Set up the detector with default parameters.
# detector = cv2.SimpleBlobDetector_create()
#
# # Detect blobs.
# keypoints = detector.detect(im)
#
# # Draw detected blobs as red circles.
# # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
# im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0, 0, 255),
#                                       cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#
# # Show keypoints
# cv2.imshow("Keypoints", im_with_keypoints)
# cv2.waitKey(0)

from scipy.signal import filtfilt, butter

order = 2  # second order filter
fs = float(30)  # sampling frequency is around 30 Hz
nyq = 0.5 * fs
lowcut = 2  # cutoff frequency at 2 Hz
low = lowcut / nyq
b, a, *_ = butter(order, [low], btype='lowpass', output='ba') # tiro en _ para evitar intellisense highlight
print("B - A:", b, "-", a)

corto = [5, 6, 1, 5, 2, 6, 2, 1, 2, 5, 6, 1]
largo = corto * 10000000

xFiltered = filtfilt(b, a, corto)
xDroneFiltered = xFiltered[-1]
print(xDroneFiltered)

xFiltered = filtfilt(b, a, largo[-10:])
xDroneFiltered = xFiltered[-1]
print(xDroneFiltered)

print("---------------------------------------")

a = np.array([2,3,4])
z, x, c = a
print(type(a), a)
print(type(z), z)
Z = int(z)
print(type(Z), Z)