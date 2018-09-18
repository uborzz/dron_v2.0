import numpy as np
import cv2

upper = np.array([7, 255, 255])
lower = np.array([173, 55, 55])

if upper[0] >= lower[0]:

    print("OK")

else:
    lower1 = lower.copy()
    lower1[0] = 0
    upper1 = upper.copy()
    lower2 = lower.copy()
    upper2 = upper.copy()
    upper2[0] = 180
    print(lower1, upper1, lower2, upper2)
    print(type(lower1))