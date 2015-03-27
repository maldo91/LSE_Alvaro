from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import socket
import threading
import sys
class SensorThread(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
	def run(self):
		sc, addr = s.accept()
		received = sc.recv(1024)
		receivedContent = received.split(',',itemsReceived)
		sensorID = receivedContent[1]
		new = 1
		lock.acquire()
		if  len(sensorTable) != 0:  #Not empty table
			element = 0;
			while (element < len(sensorTable)):
				elementContent = sensorTable[element].split(',',itemsReceived)
				sensorIDTable = elementContent[1]
				if sensorID == sensorIDTable:
					sensorTable.pop(element) #Delete old value
					sensorTable.insert(element,received)
					new = 0
				element = element + 1
			if new: #New element
				sensorTable.append(received)
		else:
			sensorTable.append(received)
		lock.release()
		#print  "Conexion:",recived
		sc.close()


class SensorMainThread(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
	def run(self):
		while True:
			#print "Another thread"
			t = SensorThread()
			t.start()
			t.join()

class myHandler(BaseHTTPRequestHandler):
	
	#Handler for the GET requests
	def do_GET(self):
		#print self.path
		if self.path.endswith('.html') or self.path == "/":
			self.send_response(200)
			self.send_header('Content-type','text/html')
			self.end_headers()
			if (self.path == "/") or (self.path == "/index.html") :
				f = open("index.html", "r+")
			elif (self.path == "/cocina.html"):
				f = open("cocina.html", "r+")
			elif (self.path == "/bano.html"):
				f = open("bano.html", "r+")
			elif (self.path == "/salon.html"):
				f = open("salon.html", "r+")
			elif (self.path == "/dormitorio.html"):
				f = open("dormitorio.html", "r+")
			else:
				f = open("pasillo.html", "r+")
			content = f.read()
			f.close()
			###################################################################
			
			###################################################################
			listContent = content.split("Estos son los valores de los sensores:",1)
			#Last action removes the text between quotes. It must be added again
			listContent.insert(1,"Estos son los valores de los sensores:"+"<br><br>")
			i = 2
			lock.acquire()
			for element in sensorTable:
				sensorContent = element.split(',',itemsReceived)		
				listContent.insert(i,"Sensor: "+ sensorContent[1] +"<br>")
				listContent.insert(i+1,"&nbsp;&nbsp;&nbsp;Temperatura: "+ sensorContent[3] +" &deg;C<br>")	
				listContent.insert(i+2,"&nbsp;&nbsp;&nbsp;Luminosidad: "+ sensorContent[5] +" cd<br>")	
				listContent.insert(i+3,"&nbsp;&nbsp;&nbsp;Ruido: "+ sensorContent[7] +" dB<br><br>")		
				i = i + 4
			lock.release()
			page = ""
			for element in listContent:
				page = page + element
			self.wfile.write(page)
		if self.path.endswith('.css'):
			self.send_response(200)
			self.send_header('Content-type','text/html')
			self.end_headers()
			f = open("estilos.css", "r")
			self.wfile.write(f.read())
			f.close()
		if	self.path.endswith('.jpg'):
			self.send_response(200)
                        self.send_header("Content-type", "image/jpg")
                        self.end_headers()
			f = open("."+self.path, "rb")
			self.wfile.write(f.read())
			f.close()

		return

class WebMainThread(threading.Thread):
	def __init__(self):	
		threading.Thread.__init__(self)
	def run(self):
		#Create a web server and define the handler to manage the
		#incoming request
		server = HTTPServer(('', WEB_PORT_NUMBER), myHandler)
		print 'Started httpserver on port ' , WEB_PORT_NUMBER
	
		#Wait forever for incoming htto requests
		server.serve_forever()


if(len(sys.argv) == 2):
	sensorTable = []
	lock = threading.Lock()
	WEB_PORT_NUMBER = 8080
	itemsReceived = 10
	s = socket.socket()
	myIP = str(sys.argv[1])
	myIP_PORT = myIP + ":" + str(WEB_PORT_NUMBER)
	s.bind((myIP, 9999))
	s.listen(1)

	tSensor = SensorMainThread()
	tWeb = WebMainThread()
	tSensor.start()
	tWeb.start()
	tSensor.join()
	tWeb.join()
else:
	print "Invalid number of arguments"
