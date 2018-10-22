from datetime import datetime
import globales as gb
from rady_configurator import Configurator

## prueba formateo fecha -> str

a = "{:%d_%H_%M}.json".format(datetime.now())

print(a)

## prueba rotacion - creacion ficheros carpeta
conf = Configurator()
print(gb.config_activa)

while True:
    conf.select_another_config_file()
    a = input()
    if a == "a":
        conf.create_new_config_file()
