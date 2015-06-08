import serial
import time
import math
import select
import json

AX_WRITE = 0x03
AX_READ = 0x02
RX_MODE = 0
TX_MODE = 1
MAX_WAIT = 3
SERVOS = 4		#Nr of servos
ENDCHAR = '$'
IDS=((4,18,1,13))
print IDS
s=serial.Serial()

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
	help=float(float(angle<<10)/300)
	lowbyte=int(help%256)
	highbyte=int(help)>>8
	return setReg(ID,30,((lowbyte,highbyte)))

#Returns position of given servo(ID)
def getPosition(ID):
	(ids,err,lowbyte,highbyte) = getReg(ID,36,2)
	return (highbyte<<8)+lowbyte

init()

writeFile(TX_MODE)
testid = 18
setReg(testid,5,((50,)))
time.sleep(0.0000001)
pos = 0
idnr = 0
while(1):
	move(IDS[idnr],pos)
	time.sleep(0.000001)
	pos = (pos + 1)%90
	print getPosition(IDS[idnr])
	time.sleep(0.000001)
	idnr = (idnr +1)%4
#	move(testid,60)
#	time.sleep(0.000001)
#	print getPosition(testid)
