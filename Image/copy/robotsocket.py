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
PHI1 = 0
PHI2 = 0
HZ = 0
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

	#Sets digital pin2 to output and receives signal from gpio14
	File = open('/sys/class/gpio/gpio31/value', 'w')
	File.write('0')
	File.close()
	File = open('/sys/class/gpio/gpio14/direction', 'w')
	File.write('out')
	File.close()

	#Creates server socket
	print "hostname=",socket.gethostname()
	serversocket.settimeout(0.0)
	serversocket.bind(('192.168.1.2', 63000))
	print "socketname=",serversocket.getsockname()
	print "addrinfo=",socket.getaddrinfo('192.168.1.2',63000)
	serversocket.listen(1)

#Select read or write mode
def writeFile(data):
	File = open('/sys/class/gpio/gpio14/value', 'w')
	File.write(str(data))

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
			chunk = clientsocket.recv(64)
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
	

def generate():
	T = time.time()
	frontright = int(60+30*math.cos(HZ*2*math.pi*T))								#SERVO 4
	frontleft = int(60-30*math.cos(HZ*2*math.pi*T + PHI2))							#SERVO 18
	backright = int(60+30*math.cos(HZ*2*math.pi*T + PHI1))							#SERVO 1
	backleft = int(60-30*math.cos(HZ*2*math.pi*T + PHI2 + PHI1))					#SERVO 13
	updateServos(frontright,frontleft,backright,backleft)
	
init()
writeFile(TX_MODE)
setStraight()
time.sleep(1)
for i in IDS:
	feedback[i]=getPosition(i)
responspacket = json.dumps(feedback)
print "responspacket=",responspacket
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
			PHI1 = data['22']
			PHI2 = data['23']
			if data['20']:							#True = PCGen / False = GalileoGen
				frontright = data['4']								#SERVO 4
				frontleft = data['18']								#SERVO 18
				backright = data['1']								#SERVO 1
				backleft = data['13']								#SERVO 13
				updateServos(frontright,frontleft,backright,backleft)
				responspacket = json.dumps(feedback)
				print "responspacket=",responspacket
			else:
				generate()
				responspacket = json.dumps(feedback)
				print "responspacket=",responspacket
			s.flush()
	except:
		generate()
		responspacket = json.dumps(feedback)
		print "responspacket=",responspacket

clientsocket.close()
serversocket.close()
s.close()


