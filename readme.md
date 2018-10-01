# instalación
Ubuntu 14+
Python 3.4+

## Instala python y git
```bash
sudo apt-get update
sudo apt-get install python3
sudo apt-get install git
```

## Crea una carpeta y clona el repositorio dentro
```bash
mkdir dron
cd dron
git clone https://github.com/uborzz/dron_v2.0
```

## Instala pip y virtualenv
```bash
sudo apt-get install python3-pip
sudo pip3 install virtualenv
```

## Crea y activa el entorno virtual con python 3
```bash
virtualenv /usr/bin/python3.4 dron-venv
source dron-venv/bin/activate
```

## Instala paquetes de python necesarios
```bash
pip3 install -r dron_v2.0/requirements.txt
```


# configuración
A mano de momento, en el fichero main.py

## seleccionar cámara
src=0 para la primera cámara, src=1 la segunda...
```python
cam = Stream(**src=0**, resolution=(width, height), framerate=fps_camera).start()
```

seleccionar puerto arduino
```python
midron = dron.create_dron(**"/dev/ttyUSB0"**, simulated=False)
```