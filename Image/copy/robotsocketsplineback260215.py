import serial
import time
import math
import socket
import select
import json

AX_WRITE = 0x03
AX_READ = 0x02
RX_MODE = 0
TX_MODE = 1
MAX_WAIT = 3
SERVOS = 4		#Nr of servos
ENDCHAR = '$'

#Parameters for all legs
HZ = 0.0			#frequency
MX = 0.125
MY = 0.125


#Parameters for front legs

#Parameters for back legs

#Front left
FLX0 = 0.15
FLY0 = 1.0

FLX1 = 0.35
FLY1 = -1.0

#Front right
FRX0 = 0.15
FRY0 = 1.0

FRX1 = 0.35
FRY1 = -1.0

#Back left
BLX0 = 0.15
BLY0 = -1.0

BLX1 = 0.35
BLY1 = 1.0

#Back right
BRX0 = 0.15
BRY0 = -1.0

BRX1 = 0.35
BRY1 = 1.0


feedback = {}
IDS=((4,18,1,13))
print IDS
s=serial.Serial()
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def init():
	#Open serial communication on pin 0(RX) and 1(TX)
	s.baudrate = 115200
	s.port = "/dev/ttyS0"
	s.timeout = 0
	print s
	s.open()
	
	#Jumper pin 8 to pin 9
	#Connect to set HZ to 2.00 at startup
	File = open('/sys/class/gpio/gpio26/direction', 'w')
	File.write('out')
	File.close()
	File = open('/sys/class/gpio/gpio19/direction', 'w')
	File.write('out')
	File.close()
	File = open('/sys/class/gpio/gpio19/value', 'w')
	File.write('0')
	File.close()
        File = open('/sys/class/gpio/gpio19/direction', 'w')
        File.write('in')
        File.close()
	File = open('/sys/class/gpio/gpio26/value', 'w')
	File.write('1')
	File.close()
        File = open('/sys/class/gpio/gpio19/value', 'r')
        input = int(File.read())

	#Sets digital pin2 to output and receives signal from gpio14
	File = open('/sys/class/gpio/gpio31/value', 'w')
	File.write('0')
	File.close()
	File = open('/sys/class/gpio/gpio14/direction', 'w')
	File.write('out')
	File.close()

	#Sets A0-A3 to ADC input
	File = open('/sys/class/gpio/gpio37/value', 'w')
	File.write('0')
	File.close()
	File = open('/sys/class/gpio/gpio36/value', 'w')
	File.write('0')
	File.close()
	File = open('/sys/class/gpio/gpio23/value', 'w')
	File.write('0')
	File.close()
	File = open('/sys/class/gpio/gpio22/value', 'w')
	File.write('0')
	File.close()

	#Creates server socket
	print "hostname=",socket.gethostname()
	serversocket.settimeout(0.0)
	serversocket.bind(('192.168.1.2', 63000))
	print "socketname=",serversocket.getsockname()
	print "addrinfo=",socket.getaddrinfo('192.168.1.2',63000)
	serversocket.listen(1)
	
	return input*2.0



#Select read or write mode
def writeFile(data):
	File = open('/sys/class/gpio/gpio14/value', 'w')
	File.write(str(data))

#Reads sensordata and return list
def readSensors():
        sensordata = list()
	File0 = open('/sys/bus/iio/devices/iio:device0/in_voltage0_raw', 'r')	#Back left
        File1 = open('/sys/bus/iio/devices/iio:device0/in_voltage1_raw', 'r')	#Front left
        File2 = open('/sys/bus/iio/devices/iio:device0/in_voltage2_raw', 'r')	#Back right
        File3 = open('/sys/bus/iio/devices/iio:device0/in_voltage3_raw', 'r')	#Front right
	sensordata.append(File0.read())
	sensordata.append(File1.read())
	sensordata.append(File2.read())
	sensordata.append(File3.read())
	return sensordata

#Returns status packet
def statusReturn(ID, NUMBER):   #expects RX_MODE
	wait=0
	#important to wait at least one time for respons
	while s.inWaiting()==0 and wait<MAX_WAIT:
		time.sleep(0.001)	
		wait = wait+1
	if s.inWaiting()!=NUMBER:
		vals = list()
		vals.append(ID)
		vals.append(10)
		for i in range(0,NUMBER-6):
			vals.append(0)
		s.flushInput()	
		return vals
	else:
		vals = list()
		statuspacket = s.read(NUMBER)
		vals.append(ord(statuspacket[2]))
		vals.append(ord(statuspacket[4]))
		for i in range(5,5+NUMBER-6):
			vals.append(ord(statuspacket[i]))
		return vals

#Sets given values starting in reg
def setReg(ID,reg,values):
	length = 3 + len(values)
	checksum = 255-((ID+length+AX_WRITE+reg+sum(values))%256)
	message = chr(0xFF)+chr(0xFF)+chr(ID)+chr(length)+chr(AX_WRITE)+chr(reg)
	for val in values:
		message = message + chr(val)
	message = message + chr(checksum)
	s.write(message)

#Gets rlength registers starting from regstart
def getReg(ID, regstart, rlength):
	checksum = 255 - ((6 + ID + regstart + rlength)%256)
	s.write(chr(0xFF)+chr(0xFF)+chr(ID)+chr(0x04)+chr(AX_READ)+chr(regstart)+chr(rlength)+chr(checksum))
	writeFile(RX_MODE)
	vals = list()
	vals = statusReturn(ID, rlength+6)
	writeFile(TX_MODE)
	return vals

#Sets STATUS RETURN LEVEL
def setReturn(ID, value):
	return setReg(ID, 16, [value])

#Sets servo(ID) to given angle (0 to 300)
def move(ID,angle):
	time.sleep(0.0000001)
	help=float(float(angle<<10)/300)
	lowbyte=int(help%256)
	highbyte=int(help)>>8
	return setReg(ID,30,((lowbyte,highbyte)))

#Returns position of given servo(ID)
def getPosition(ID):
	time.sleep(0.0000001)
	(ids,err,lowbyte,highbyte) = getReg(ID,36,2)
	return (highbyte<<8)+lowbyte

#Send data via socket
def robotsend(data):
	totalsent = 0
	data = data + ENDCHAR
	while totalsent < len(data):
		sent = clientsocket.send(data[totalsent:])
		if sent == 0:
			clientsocket.close()
			print "Client disconnected"
			raise RuntimeError("socket connection broken")
		totalsent = totalsent + sent

#Read data from socket
def robotreceive(nextpart):
	chunks = []
	chunks.append(nextpart)
	endcharrecv = False
	while not endcharrecv:
			chunk = clientsocket.recv(512)
			if chunk == '':
				clientsocket.close()
				print "Client disconnected"
				raise RuntimeError("socket connection broken")
			endcharrecv = '$' in chunk
			if endcharrecv:
				parts = chunk.split('$')
				chunks.append(parts[0])
				extra = parts[1:-2]
				next = parts[-1]
			else:
				chunks.append(chunk)
	return (chunks, next, extra)	
		
def setStraight():
	for i in IDS:
		move(i,60)
		time.sleep(0.2)

def moveBack():
	for i in IDS:
		move(i,30)
		time.sleep(0.2)

def moveForward():
	for i in IDS:
		move(i,90)
		time.sleep(0.2)	

def updateServos(frontright,frontleft,backright,backleft):
	move(4,frontright)
	feedback[4]=getPosition(4)
	move(18,frontleft)
	feedback[18]=getPosition(18)
	move(1,backright)
	feedback[1]=getPosition(1)
	move(13,backleft)
	feedback[13]=getPosition(13)
	distances = readSensors()
	feedback[2] = distances[2]	#BR (1)
	feedback[5] = distances[3]	#FR (4)
	feedback[14] = distances[0]	#BL (13)
	feedback[19] = distances[1]	#FL (18)

def generate():
	T = time.time()
	frontright = int(60.0+30.0*hermiteSpline(T, FRX0, FRY0, FRX1, FRY1, HZ))							#SERVO 4
	frontleft = int(60.0-30.0*hermiteSpline(T, FLX0, FLY0, FLX1, FLY1, HZ))							#SERVO 18
	backright = int(60.0+30.0*hermiteSpline(T, BRX0, BRY0, BRX1, BRY1, HZ))							#SERVO 1
	backleft = int(60.0-30.0*hermiteSpline(T, BLX0, BLY0, BLX1, BLY1, HZ))								#SERVO 13
	updateServos(frontright,frontleft,backright,backleft)

def hermiteSpline(time, x0, y0, x1, y1, freq):
	if freq == 0.0:
		return 0.0
	time = time % (1.0/freq)
	if time > 0 and time <= x0:
		t = time/x0
		return hermite(t,0,y0,MX,MY)
	elif time > x0 and time <= x1:
		t = (time - x0)/(x1-x0)
		return hermite(t,y0,y1,MX,MY)
	elif time > x1 and time <= (1.0/freq):
		t = (time - x1)/((1.0/freq)-x1)
		return hermite(t,y1,0,MX,MY)
	else:
		print "this isn't supposed to happen"

def hermite(h,k0,k1,m0,m1):
	return (2*h*h*h-3*h*h+1)*k0+(-2*h*h*h+3*h*h)*k1+(h*h*h-2*h*h+h)*m0+(h*h*h-h*h)*m1
	
HZ = init()
writeFile(TX_MODE)
setStraight()
time.sleep(1)
for i in IDS:
	feedback[i]=getPosition(i)
responspacket = json.dumps(feedback)
#print "responspacket=",responspacket
s.flush()

while 1:
	try:
		(clientsocket, address) = serversocket.accept()
		print "A client connected"
		print "socket=",clientsocket.getsockname()
		print "address=",address
		while 1:
			robotsend(responspacket)
			nextpart = ""
			(part,nextpart,extra) = robotreceive(nextpart)
			packet = ''.join(part)
			data = json.loads(packet)
			HZ = data['21']							#updateparams
			FLX0 = data['22']
			FLY0 = data['23']
			FLX1 = data['24']
			FLY1 = data['25']
			FRX0 = data['26']
			FRY0 = data['27']
			FRX1 = data['28']
			FRY1 = data['29']
			BLX0 = data['30']
			BLY0 = data['31']
			BLX1 = data['32']
			BLY1 = data['33']
			BRX0 = data['34']
			BRY0 = data['35']
			BRX1 = data['36']
			BRY1 = data['37']
			if data['20']:							#True = PCGen / False = GalileoGen
				frontright = data['4']								#SERVO 4
				frontleft = data['18']								#SERVO 18
				backright = data['1']								#SERVO 1
				backleft = data['13']								#SERVO 13
				updateServos(frontright,frontleft,backright,backleft)
				responspacket = json.dumps(feedback)
#				print "responspacket=",responspacket
			else:
				generate()
				responspacket = json.dumps(feedback)
#				print "responspacket=",responspacket
			s.flush()
	except:
		generate()
		responspacket = json.dumps(feedback)
#		print "responspacket=",responspacket

clientsocket.close()
serversocket.close()
s.close()


