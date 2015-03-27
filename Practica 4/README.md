# Práctica 4
El fichero del sensor es el sensor.py
La forma de llamarlo es:
	"python sensor.py [ID] [nodoCentralIP] [adquisitionTime] [place]"
	donde ID es el nombre del sensor, nodoCentralIP es la IP del servidor al que envian la información los sensores, adquisitionTime es el tiempo entre adquisiciones de los datos y place es el lugar donde estará el sensor (todavía no implementado)

	
	Ejemplo:
	python sensor.py 1 138.4.216.4 2 cocina

El fichero del sensor central al que enviían la información los sensores y que ejecuta tmb el web server  es el nodoCentral.py
La forma de llamarlo es:
	"python nodoCentral.py [nodoCentralIP] "
	donde  nodoCentralIP es la IP del servidor al que envian la información los sensores y dnd se ejecuta tmb el web server
	Ejemplo
	python nodoCentral.py 138.4.216.4

