#Author: Peter Timmerman
#Contact: peter.timmerman0@gmail.com
import serial
import time
import math
import socket
import select
import json
import collections
import numpy as np

AX_WRITE = 0x03
AX_READ = 0x02
RX_MODE = 0
TX_MODE = 1
MAX_WAIT = 5
SERVOS = 4		#Nr of servos, code is for four servos!
ENDCHAR = '$'
POSFB = False
RUNNING = False
OFFSETTIME = 0.0
WAITTIME = 0.25

#Parameters for all legs
HZ = 0.0			#frequency
OFF = 60.0
A = 30.0

#Parameters for front legs
FX0 = 0.0
FY0 = -0.89
FX1 = 0.66
FY1 = 0.5

FMX0 = 0.14
FMY0 = 0.5
FMX1 = 0.14
FMY1 = 0.5

FD = 0.0

#Parameters for back legs
BX0 = 0.00
BY0 = 0.64
BX1 = 0.49
BY1 = -1.03

BMX0 = 0.68
BMY0 = 0.5
BMX1 = 0.68
BMY1 = 0.5

BD = 0.0

#Front left
#Front right
#Back left
#Back right

feedback = {}
IDS=((18,6,13,2)) 	#SERVO IDS, ORDER=((FL, FR, BL, BR)) IMPORTANT!!
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
        File = open('/sys/class/gpio/gpio42/value', 'w')
        File.write('1')
        File.close()
        File = open('/sys/class/gpio/gpio16/direction', 'w')
        File.write('out')
        File.close()
        File = open('/sys/class/gpio/gpio16/value', 'w')
        File.write('0')
        File.close()
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
	
        File = open('/sys/class/gpio/gpio26/value', 'w')
        File.write('0')
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
	
	File = open('/sys/class/gpio/gpio16/value', 'w')
	File.write('1')
	File.close()
	File = open('/sys/class/gpio/gpio19/value', 'r')
	input2 = int(File.read())

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
	
	selectParams(input, input2)	

def selectParams(nr,nr2):
	global FY0
	global FX1
	global FY1
	global FMX0
	global FMX1
	global BY0
	global BX1
	global BY1
	global BMX0
	global BMX1
	global HZ
	global WAITTIME
        if nr==1:
                FY0 = -1.01
                FX1 = 0.34
                FY1 = 0.91
                FMX0 = 0.19
                FMX1 = 0.19
                BY0 = 0.8
                BX1 = 0.57
                BY1 = -0.96
                BMX0 = 0.25
                BMX1 = 0.25
                HZ = 2.2
                WAITTIME = 0.12
        elif nr2==1:
		FY0 = -0.89
		FX1 = 0.66
		FY1 = 0.5
		FMX0 = 0.14
		FMX1 = 0.14
		BY0 = 0.64
		BX1 = 0.49
		BY1 = -1.03
		BMX0 = 0.68
		BMX1 = 0.68
		HZ = 2.2
		WAITTIME = 0.25
	else:
		HZ = 0.0		

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
	while s.inWaiting()<NUMBER-2 and wait<MAX_WAIT:
		time.sleep(0.0001)	
		wait = wait+1
	inbuffer = s.inWaiting()
	statuspacket = s.read(inbuffer)	
	vals = list()
	if inbuffer>=NUMBER-2 and inbuffer<=NUMBER and ord(statuspacket[2+inbuffer-NUMBER])==ID and ord(statuspacket[4+inbuffer-NUMBER])==0:
		vals.append(ord(statuspacket[2+inbuffer-NUMBER]))
		vals.append(ord(statuspacket[4+inbuffer-NUMBER]))
		valslength = NUMBER-6	#2startbytes, ID, Length, Error and checksum = 6
		for i in range(5+inbuffer-NUMBER,5+inbuffer-NUMBER+valslength):
			vals.append(ord(statuspacket[i]))
		return vals
	else:
		vals.append(ID)
		vals.append(10)
		for i in range(0,NUMBER-6):
			vals.append(0)
		s.flushInput()
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
	if err==10:
		return 200
	else:
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
                time.sleep(0.15)

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
	if POSFB:
		feedback[IDS[1]]=getPosition(IDS[1])
	move(IDS[0],frontleft)
	if POSFB:
		feedback[IDS[0]]=getPosition(IDS[0])
	move(IDS[3],backright)
	if POSFB:
		feedback[IDS[3]]=getPosition(IDS[3])
	move(IDS[2],backleft)
	if POSFB:
		feedback[IDS[2]]=getPosition(IDS[2])
	distances = readSensors()
	feedback[258] = distances[2]	#BR IDS[3]
	feedback[256] = distances[3]	#FR IDS[1]
	feedback[257] = distances[0]	#BL IDS[2]
	feedback[255] = distances[1]	#FL IDS[0]

#Generates new angles for servos using hermitespline
#Updates servos
def generate():
	T = WAITTIME + time.time() - OFFSETTIME
	frontright = int(OFF+A*hermiteSpline(T, FX0, FY0, FX1, FY1, FMX0, FMY0, FMX1, FMY1, HZ, FD))							#SERVO 6
	frontleft = int(OFF-A*hermiteSpline(T, FX0, FY0, FX1, FY1, FMX0, FMY0, FMX1, FMY1, HZ, FD))							#SERVO 18
	backright = int(OFF+A*hermiteSpline(T, BX0, BY0, BX1, BY1, BMX0, BMY0, BMX1, BMY1, HZ, BD))							#SERVO 2
	backleft = int(OFF-A*hermiteSpline(T, BX0, BY0, BX1, BY1, BMX0, BMY0, BMX1, BMY1, HZ, BD))							#SERVO 13
	updateServos(frontright,frontleft,backright,backleft)

#Generates y-values with given parameters and cubic hermite interpolation
def hermiteSpline(time, x0, y0, x1, y1, mx0, my0, mx1, my1, freq, delay):
	if freq == 0.0:
		return 0.0
	time = (time + delay) % (1.0/freq)
	time = time/(1.0/freq)
	if time<x1:					#First segment time = [0:x1]
		t = reversehermite(0.0,x1,mx0,mx1,time)
 		return hermite(t,y0,y1,my0,my1)
	elif time>=x1:					#Second segment time = [x1:1[
		t = reversehermite(x1,1.0,mx1,mx0,time)
		return hermite(t,y1,y0,my1,my0)
	else:
		print "This shouldn't happen!"	

#Hermite interpolation
def hermite(h,k0,k1,m0,m1):
	return (2*h*h*h-3*h*h+1)*k0+(-2*h*h*h+3*h*h)*k1+(h*h*h-2*h*h+h)*m0+(h*h*h-h*h)*m1

#Reverse Hermite interpolation
def reversehermite(x0,x1,mx0,mx1, t):
	if t==0:
		return 0.0
	a = (2*x0-2*x1+mx0+mx1)/t
	b = (-3*x0+3*x1-2*mx0-mx1)/t
	c = (mx0)/t
	d = (x0)/t-1
	roots = []
	roots = np.roots([a,b,c,d])
	for root in roots:
		if not(isinstance(root,complex)):
			if root >= 0.0 and root <= 1.0:
				return root
		elif root.imag == 0.0:
			if root.real >= 0.0 and root.real <= 1.0:
				return root.real
	print "This shouldn't happen", t, roots
	return 0.0

	
init()			#Frequency can be set to 0.0 or 2.0 using jumper in GPIO 8 and 9
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
OFFSETTIME = time.time()
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
			FY0 = data['256']
			FX1 = data['257']
			FY1 = data['258']
			FMX0 = data['259']
			FMX1 = data['259']
			BY0 = data['260']
			BX1 = data['261']
			BY1 = data['262']
			BMX0 = data['263']
			BMX1 = data['263']
			POSFB = data['273']
			USEMAGIC = data['274']
			if data['272']:							#True = PCGen / False = GalileoGen
				frontright = int(data[str(IDS[1])])								#SERVO 4
				frontleft = int(data[str(IDS[0])])								#SERVO 18
				backright = int(data[str(IDS[3])])								#SERVO 1
				backleft = int(data[str(IDS[2])])								#SERVO 13
#				print frontleft, " - ", frontright, " - ", backleft, " - ", backright
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
		feedback[255]=IDS[0]
		feedback[256]=IDS[1]
		feedback[257]=IDS[2]
		feedback[258]=IDS[3]
		responspacket = json.dumps(feedback)
#		print "responspacket=",responspacket

clientsocket.close()
serversocket.close()
s.close()


                                             