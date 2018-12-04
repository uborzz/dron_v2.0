a = [a for a in range(10)]
print(a)
a[-5:] = [60 for i in range(5)]     # vale

a[-2:] = [333] * 2     # vale
print(a)

import rady_functions
arr = rady_functions.rady_ring(5)
print(arr)

arr.extend(2)
arr.extend(4)
arr.extend(5)
arr.extend(2)
print(arr)
print(arr.mean())
arr.extend(10)
arr.extend(10)
arr.extend(10)
arr.extend(10)
print(arr)
print(arr.mean())
print(arr.get())
print(arr.get())
arr.extend(4)
print(arr.get_last())
print(arr)
print(arr.mean())

import rady_functions as rfs

angulos = [1, 357, 359, 0, 1]
print(rfs.meanangle(angulos))