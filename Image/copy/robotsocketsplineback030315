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
SERVOS = 4		#Nr of servos, code is for four servos!
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
IDS=((18,4,13,2)) 	#SERVO IDS, ORDER=((FL, FR, BL, BR))
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
	sensordata.append(int(File0.read()))
	sensordata.append(int(File1.read()))
	sensordata.append(int(File2.read()))
	sensordata.append(int(File3.read()))
	return sensordata

#Returns status packet
def statusReturn(ID, NUMBER):   #expects RX_MODE
	wait=0
	#important to wait at least one time for respons
	while s.inWaiting()==0 and wait<MAX_WAIT:
		time.sleep(0.0001)	
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

def setBack():
	for i in IDS:
		move(i,30)
		time.sleep(0.2)

def setForward():
	for i in IDS:
		move(i,90)
		time.sleep(0.2)	

#Updates servos with new angle
#Gets position of each servo
#Reads sensor output
#Feedback is stored in dictionary "feedback"
def updateServos(frontright,frontleft,backright,backleft):
	move(IDS[1],frontright)
	feedback[IDS[1]]=getPosition(IDS[1])
	move(IDS[0],frontleft)
	feedback[IDS[0]]=getPosition(IDS[0])
	move(IDS[3],backright)
	feedback[IDS[3]]=getPosition(IDS[3])
	move(IDS[2],backleft)
	feedback[IDS[2]]=getPosition(IDS[2])
	distances = readSensors()
	feedback[258] = distances[2]	#BR IDS[3]
	feedback[256] = distances[3]	#FR IDS[1]
	feedback[257] = distances[0]	#BL IDS[2]
	feedback[255] = distances[1]	#FL IDS[0]

#Generates new angles for servos using hermitespline
#Updates servos
def generate():
	T = time.time()
	frontright = int(60.0+30.0*hermiteSpline(T, FRX0, FRY0, FRX1, FRY1, HZ))							#SERVO 4
	frontleft = int(60.0-30.0*hermiteSpline(T, FLX0, FLY0, FLX1, FLY1, HZ))							#SERVO 18
	backright = int(60.0+30.0*hermiteSpline(T, BRX0, BRY0, BRX1, BRY1, HZ))							#SERVO 1
	backleft = int(60.0-30.0*hermiteSpline(T, BLX0, BLY0, BLX1, BLY1, HZ))								#SERVO 13
	updateServos(frontright,frontleft,backright,backleft)

#Calculates y-value with given parameters using cubic hermite interpolation
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

#Hermite interpolation
def hermite(h,k0,k1,m0,m1):
	return (2*h*h*h-3*h*h+1)*k0+(-2*h*h*h+3*h*h)*k1+(h*h*h-2*h*h+h)*m0+(h*h*h-h*h)*m1

	
HZ = init()			#Frequency can be set to 0.0 or 2.0 using jumper in GPIO 8 and 9
writeFile(TX_MODE)		#Necessary for proper operation, set to write-mode
setStraight()			#Robot stands up
time.sleep(1)
for i in IDS:			#First feedback packet with servo IDS and positions
	feedback[i]=getPosition(i)
feedback[255]=IDS[0]
feedback[256]=IDS[1]
feedback[257]=IDS[2]
feedback[258]=IDS[3]
responspacket = json.dumps(feedback)
#print "responspacket=",responspacket
s.flush()

#MAIN
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
			HZ = data['255']							#updateparams
			FLX0 = data['256']
			FLY0 = data['257']
			FLX1 = data['258']
			FLY1 = data['259']
			FRX0 = data['260']
			FRY0 = data['261']
			FRX1 = data['262']
			FRY1 = data['263']
			BLX0 = data['264']
			BLY0 = data['265']
			BLX1 = data['266']
			BLY1 = data['267']
			BRX0 = data['268']
			BRY0 = data['269']
			BRX1 = data['270']
			BRY1 = data['271']
			if data['272']:							#True = PCGen / False = GalileoGen
				frontright = data[str(IDS[1])]								#SERVO 4
				frontleft = data[str(IDS[0])]								#SERVO 18
				backright = data[str(IDS[3])]								#SERVO 1
				backleft = data[str(IDS[2])]								#SERVO 13
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


