import os

"""
preparacion para la funcion calibrate.
cambia el nombre de todas las imagenes encontradas en <directory_path> por <name_root>XX<name_sufix>
XX es la numeracion.

- rady
"""

# config
directory_path = "./captures"
name_root = "image"
name_sufix = ".jpg"

files = os.listdir(directory_path)
print(files)

count = 1
for file in files:
    id = "%02d" % count
    os.rename(directory_path + "/" + file, directory_path + "/" + name_root + id + name_sufix )
    count = count + 1
