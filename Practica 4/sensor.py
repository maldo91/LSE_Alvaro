import random
import socket
import time
import sys

class sensor:
	def __init__(self, sensorName, centralNodeIp, sleepTime, placement):
		self.sensorName = sensorName
		self.centralNodeIp = centralNodeIp
		self.sleepTime = sleepTime
		self.temperature = 0.0
		self.luminosity = 0.0
		self.noise = 0.0
		self.placement = placement
		self.socketInstance = socket.socket()
	
	def readTemperature(self):
		#Temperature sensing simulation. Generate a temperature between 15.0 C to 45.0 C
		self.temperature = round(15.0 + random.random() * 30,1)	
	def readLuminosity(self):
		#Luminosity sensing simulation.
		self.luminosity = round(0.0 + random.random() * 20,1)
	def readNoise(self):
		#Luminosity sensing simulation.Generate a noise between 20.0 dB to 110.0 dB
		self.noise = round(20.0 + random.random() * 90,1)
	
	def readData(self):
		self.readTemperature()
		self.readLuminosity()
		self.readNoise()

	def sendData(self):
		self.socketInstance = socket.socket()
		self.socketInstance.connect((self.centralNodeIp, 9999))
		#The message is in plain text (as a string). The way of building this message is "our protocol"
		message ="SensorName,"      + str(self.sensorName) + \
			",SensorTemperature," + str(self.temperature) + \
			",SensorLuminosity,"  + str(self.luminosity) + \
			",SensorNoise,"       + str(self.noise) + \
			",SensorType,"
		
		self.socketInstance.send(message)
		self.socketInstance.close()
	def sleepSensor(self):
		time.sleep(self.sleepTime)
	def run(self):
		while True:
			self.readData()
			self.sendData()
			self.sleepSensor()
if(len(sys.argv) == 5):
	mySensor = sensor(sys.argv[1],str(sys.argv[2]),int(sys.argv[3]),str(sys.argv[4])) #sensorName, centralNodeIp, sleepTime, placement
	mySensor.run()
else:
	print "Invalid number of arguments"
