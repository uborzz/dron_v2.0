from datetime import datetime
import globals as gb
from rady_configurator import Configurator

a = "{:%d_%H_%M}.json".format(datetime.now())

print(a)


conf = Configurator()
print(gb.config_activa)

while True:
    conf.select_another_config_file()
    a = input()
    if a == "a":
        conf.create_new_config_file()
