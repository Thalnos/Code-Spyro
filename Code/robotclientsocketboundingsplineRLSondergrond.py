#Author: Peter Timmerman
#
#Contact: peter.timmerman0@gmail.com
#
#This code is designed to connect to the Intel Galileo on the Puppy robot. It generates a pattern for the servo's. This is done with a spline.
#The pattern is designed for a bounding gait. Front legs work as a pair and back legs as well. Front and back work opposite.
#Morphological computation is added using RLS.
#Terrain classification is added with Linear Regression
#


import socket
import time
import math
import select
import json
import collections
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
from threading import Thread
from matplotlib.widgets import Slider, Button, CheckButtons
from matplotlib.text import Text

#
CONFIGURATIE = "VASTVEER"	#Used for logging in the folder with given name (folder must exist!)
STARTRLS = True			#File RLSMagic.txt is read at startup and robot runs autonomously
STARTONDERGROND = True		#Terrain classification initializes at startup

MSELOG = False			#Log MSE during training
LOGGER = False			#Log control signal and feedback
SAVEMAGIC = True		#Save W-matrix after training		
FEEDBACK = True		#True: Galileo sends feedback from servo's | False: Galileo doesn't send feedback
GENERATOR = True	#True: PC sends exact angles for servo's | False: PC sends parameters for angle function
POSFB = False		#Enables/Disables position feedback from servo's
#



ENDCHAR = '$'			#endchar for socketcommunication
THREADSTOP = False		#Used to close all threads at the end
USEMAGIC = False		#Used to let the Galileo do the training calculations (not implemented)
MAGICUPDATED = False		#True when robot runs autonomously
HALFWAYPOINT = False		#Halfway through learning
RUNNING = False			#Is the robot running or not

MEMORY = 20		#Number of previous timesteps
RLS = True		#Using RLS instead of linear regression (for locomotion, terrain classification is always linear regression)	
BIAS = 1		#Using one BIAS signal with feedback
OFFSETTIME = 0.0	#Updated when the robot starts running
	
NORMALIZECOUNTER = 0	#Used during normalization
NORMALIZED = False	#Set to true when normalization ends
STARTMAGIC = False	#Set to true when learning starts
LEARNSTART = 0.0	#Time when learning starts

SENSAVG = [0.0,0.0,0.0,0.0]	#Averages for the sensors
SENSVAR = [0.0,0.0,0.0,0.0]	#Standard deviations for the sensors


INPUTS = 4		#4 servoinputs
OUTPUTS = 4		#4 feedbacks per timestep

DATASIZE = 500		#Buffer size for locomotion (only important for linear regression)
LEARNTIME = 10		#Time the robot learns in seconds

MAGICW = [[0.0 for i in range(0,(OUTPUTS*(MEMORY+1)+BIAS))] for j in range(0,INPUTS)]		#The magic matrix for locomotion
												#W = y.x+
INPUTdatabuffer = collections.deque([[0.0]*INPUTS]*DATASIZE, DATASIZE)				#y [INPUTS*DATASIZE]
OUTPUTdatabuffer = collections.deque([[0.0]*(OUTPUTS*(MEMORY+1)+BIAS)]*DATASIZE, DATASIZE)	#x [(OUTPUTS(MEMORY+1)+BIAS)*DATASIZE]
lastoutput = collections.deque([[0.0]*OUTPUTS]*MEMORY, MEMORY)					#Memory buffer to store previous outputs
PRECISION = np.matlib.identity(OUTPUTS*(MEMORY+1)+BIAS)						#P matrix (look up formula)

TYPES = 5		#Terrain types
BSIZE = 500		#Buffer size for terrain classification, determines how long the robot learns on each terrain type
AVGBUF = 88.0		#Averaging terrain classification (was 4 periods more or less, can differ when using another computer)
BUF = TYPES*BSIZE		#Buffer size per floor type
GRONDUPDATECOUNTER = 0	#used for learning terrain
GRONDMAGICUPDATED = False	#True when the robot learned all terrains
NEWGROUND = False		#Used for logging
GROND = -1			#Used as index for GROUNDS
UPDATED = np.zeros(TYPES)	#Used during learning terrain
ONDERGRONDINPUT = collections.deque([[0.0]*TYPES]*BUF, BUF)				#y
ONDERGRONDOUTPUT = collections.deque([[0.0]*(OUTPUTS*(MEMORY+1)+BIAS)]*BUF, BUF)	#x
ONDERGRONDMAGIC = [[0.0 for i in range(0,(OUTPUTS*(MEMORY+1)+BIAS))] for j in range(0,TYPES)]	#W
GROUNDS = ["Lucht","Glaswol", "Noppenplaat", "Hout", "Steen"]#, "Kiezels", "Piepschuim","Rubber" #Change this to your liking
GRONDPREDICTIEBUFFER = collections.deque([[0.0]*TYPES]*int(AVGBUF), int(AVGBUF))		#average the classification
LASTPREDICTION = -1		#Index in GROUNDS

#Parameters for all legs
HZ = 0.0			#frequency
A = 30.0			#Amplitude
OFF = 60.0			#Offset
IDS = [0,0,0,0]			#servo IDS
WAITTIME = 0.12			#When the robot is standing still and starts running it always start at a certain point on the spline curve. This determines that point 

#Parameters for front legs

FX0 = 0.0
FY0 = -0.8
FX1 = 0.34
FY1 = 1.2

FMX0 = 0.4
FMY0 = 0.5
FMX1 = 0.4
FMY1 = 0.5

FD = 0.0

#Parameters for back legs

BX0 = 0.00
BY0 = 0.7
BX1 = 0.25
BY1 = -0.9

BMX0 = 0.4
BMY0 = 0.5
BMX1 = 0.4
BMY1 = 0.5

BD = 0.0


#Front left
#Front right
#Back left
#Back right




#GUI params



#Send data via socket
#s = socket
#data = string
#blocking socket
def robotsend(s, data):
	totalsent = 0
	data = data + ENDCHAR
	while totalsent < len(data):
		sent = s.send(data[totalsent:])
		if sent == 0:
			raise RuntimeError("socket connection broken")
		totalsent = totalsent + sent


#Read data from socket
#nextpart = received part off current message
#	example
#		incoming data = full message + "$" + part of next message
# 	=> "part of next message" is saved and given as a parameter for next receive
#s = socket
#blocking socket
def robotreceive(s, nextpart):
	chunks = []
	chunks.append(nextpart)
	endcharrecv = False
	while not endcharrecv:
		chunk = s.recv(128)
		if chunk == '':
			raise RuntimeError("socket connection broken")
		endcharrecv = '$' in chunk
		if endcharrecv:
			parts = chunk.split('$')
			next = parts[-1]
			chunks.append(parts[0])
			extra = parts[1:-2]
		else:
			chunks.append(chunk)
	return (chunks, next)

#Updates global parameters
#These define the spline
def updateGlobals():

	global HZ
	global FY0
	global FX1
	global FY1
	global FMX0
	global FMX1
	global FD
	global BY0
	global BX1
	global BY1
	global BMX0
	global BMX1
	global BD

	HZ = sHZ.val
	FY0 = sFY0.val
	FX1 = sFX1.val
	FY1 = sFY1.val
	FMX0 = sFMX0.val
	FMX1 = FMX0
	FD = sFD.val
	BY0 = sBY0.val
	BX1 = sBX1.val
	BY1 = sBY1.val
	BMX0 = sBMX0.val
	BMX1 = BMX0
	BD = sBD.val
	

#Normalizes inputsignals and sensordata
def normalize():
	global SENSAVG
	global SENSVAR
	for i in range(-NORMALIZECOUNTER+1,0):
		SENSAVG[:] = SENSAVG[:] + OUTPUTdatabuffer[i][-OUTPUTS-1:-1]
	SENSAVG = np.array(SENSAVG)/NORMALIZECOUNTER
	for i in range(-NORMALIZECOUNTER+1,0):	
		SENSVAR[:] = SENSVAR[:] + (OUTPUTdatabuffer[i][-OUTPUTS-1:-1]-SENSAVG[:])*(OUTPUTdatabuffer[i][-OUTPUTS-1:-1]-SENSAVG[:])
	SENSVAR = np.sqrt(np.array(SENSVAR)/NORMALIZECOUNTER)
	print "Normalize end TIME=", time.time()


#Generates data for legs
#Makes prediction for control signal
#Logs MSE (if MSELOG = True)
#Classifies terrain and logs
#Sends data and parameters
#s = socket
def generateData(s):
	T = time.time()
	global INPUTdatabuffer
	global RUNNING
	global OFFSETTIME
	global NORMALIZECOUNTER
	global NORMALIZED
	global GRONDPREDICTIEBUFFER
	if(check.lines[0][0].get_visible() or experiment.lines[0][0].get_visible()):#If update is checked, update the global values
		updateGlobals()
		if not(RUNNING):
			OFFSETTIME = T
			print "Start experiment TIME=",time.time()
			RUNNING = True
		T = WAITTIME + T - OFFSETTIME
		if((check.lines[0][0].get_visible() or experiment.lines[0][0].get_visible()) and not(NORMALIZED) and not(HZ==0.0)):
			NORMALIZECOUNTER = NORMALIZECOUNTER + 1			#Start with the correct frequency!
			if T-WAITTIME>=(1.0/HZ)*5:
				normalize()
				NORMALIZED = True
	else:
		RUNNING = False
		NORMALIZECOUNTER = 0
	frontright = OFF+A*hermiteSpline(T, FX0, FY0, FX1, FY1, FMX0, FMY0, FMX1, FMY1, HZ, FD)							#SERVO 6
	frontleft = OFF-A*hermiteSpline(T, FX0, FY0, FX1, FY1, FMX0, FMY0, FMX1, FMY1, HZ, FD)							#SERVO 18
	backright = OFF+A*hermiteSpline(T, BX0, BY0, BX1, BY1, BMX0, BMY0, BMX1, BMY1, HZ, BD)							#SERVO 2
	backleft = OFF-A*hermiteSpline(T, BX0, BY0, BX1, BY1, BMX0, BMY0, BMX1, BMY1, HZ, BD)							#SERVO 13
	embodimentdata = np.dot(np.array(MAGICW),np.array([OUTPUTdatabuffer[-1]]).T)
	grondpredictiedata = np.dot(np.array(ONDERGRONDMAGIC),np.array([OUTPUTdatabuffer[-1]]).T)
	gronddata = []
	for i in range(0,TYPES):
		gronddata.append(float(grondpredictiedata[i]))
	GRONDPREDICTIEBUFFER.append(gronddata)
	grondavg = np.sum(np.array(GRONDPREDICTIEBUFFER).T[:],axis=1)/AVGBUF
	if GRONDMAGICUPDATED:
		m = max(grondavg)
		f = open(''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/","GROND",str(MEMORY),"-",str(TYPES),".txt"]), 'a')
		for i in range(0,TYPES):
			f.write(''.join([str(float(grondavg[i])),", "]))
			if grondavg[i]==m:
				print GROUNDS[i]
		f.write(''.join([str(time.time()),", "]))
		global NEWGROUND
		if NEWGROUND:
			f.write("NEW")
			NEWGROUND = False
		f.write("\n")

	for i in range(0,INPUTS):							#safety measures
		if embodimentdata[i]<10:
			embodimentdata[i] = 10
		elif embodimentdata[i]>110:
			embodimentdata[i] = 110

	if (magic.lines[0][0].get_visible() or experiment.lines[0][0].get_visible()) and MSELOG:
		teacher = np.array([frontleft,frontright,backleft,backright])
		estimation = np.array([float(embodimentdata[0]),float(embodimentdata[1]),float(embodimentdata[2]),float(embodimentdata[3])])
		MSE = ((estimation - teacher) ** 2).mean(axis=None)
		f = open(''.join(["METINGEN/",CONFIGURATIE,"/EMBODIMENT/","MSE",str(MEMORY),".txt"]), 'a')
		f.write(''.join([str(frontleft),", ",str(frontright),", ",str(backleft),", ",str(backright),", ",str(float(embodimentdata[0])),", ",str(float(embodimentdata[1])),", ",str(float(embodimentdata[2])),", ",str(float(embodimentdata[3])),", ",str(MSE),", ",str(time.time()),"\n"]))
	
	INPUTdatabuffer.append([frontleft,frontright,backleft,backright])		#build buffer for W matrix with generated data

	#Start using a mix of the prediction and the desired value
	#Over time the predicted signal is used more than the real value
	if HALFWAYPOINT:
		percentage = ((time.time()-LEARNSTART)/(LEARNTIME/2.0)-1.0)*100.0
		frontright = (percentage*float(embodimentdata[1]) + (100-percentage)*frontright)/100.0								#SERVO 6
		frontleft = (percentage*float(embodimentdata[0]) + (100-percentage)*frontleft)/100.0								#SERVO 18
		backright = (percentage*float(embodimentdata[3]) + (100-percentage)*backright)/100.0								#SERVO 2
		backleft = (percentage*float(embodimentdata[2]) + (100-percentage)*backleft)/100.0								#SERVO 13		

	if LOGGER:
		f = open(''.join(["METINGEN/",CONFIGURATIE,"/LOG/","feedback.txt"]), 'a')
		f.write(''.join([str(T),", ",str(frontleft),", ",str(frontright),", ",str(backleft),", ",str(backright),", ",str(embodimentdata[0]),", ",str(embodimentdata[1]),", ",str(embodimentdata[2]),", ",str(embodimentdata[3]),", "]))
	#When the learning process is finished, use 100% of the predicted signal
	if MAGICUPDATED:
		frontright = int(embodimentdata[1])								#SERVO 6
		frontleft = int(embodimentdata[0])								#SERVO 18
		backright = int(embodimentdata[3])								#SERVO 2
		backleft = int(embodimentdata[2])								#SERVO 13
	else:
		frontright = int(frontright)									#SERVO 6
		frontleft = int(frontleft)									#SERVO 18
		backright = int(backright)									#SERVO 2
		backleft = int(backleft)									#SERVO 13

	#data to be serialized and sent to Galileo. Galileo needs to use correct keys as well
	data = {IDS[3]:backright, IDS[1]:frontright, IDS[2]:backleft, IDS[0]:frontleft, 255:HZ, 256:FY0, 257:FX1, 258:FY1, 259:FMX0, 260:BY0, 261:BX1, 262:BY1, 263:BMX0, 272:GENERATOR, 273:POSFB, 274:USEMAGIC}
	packet = json.dumps(data)
	robotsend(s, packet)									#send data packet with parameters and values

#Generates y-values with given parameters and cubic hermite interpolation
#Function made for the third spline I designed (see thesis)
#time = current time
#x0 - my1 = coordinates of two control points and endpoints of tangents
#freq = working frequency
#delay = delay in seconds
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
#One dimesion
#h = parameter
#k0 - m1 = two control points and two tangents
def hermite(h,k0,k1,m0,m1):
	return (2*h*h*h-3*h*h+1)*k0+(-2*h*h*h+3*h*h)*k1+(h*h*h-2*h*h+h)*m0+(h*h*h-h*h)*m1

#Reverse Hermite interpolation
#One dimension
#t = time
#x0 - mx1 = two control points and two tangents
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

#Receives respons packet
#Puts all the feedback in the correct buffers
#s = socket
def getRespons(s, next):
	(respacket,next) = robotreceive(s, next)				#get feedback
	resp = ''.join(respacket)
	respons = json.loads(resp)
	global FLdatabuffer
	global FRdatabuffer
	global BLdatabuffer
	
	global BRdatabuffer
	global OUTPUTdatabuffer
	global lastoutput		

	FLdatabuffer.append(respons['255'])
	FRdatabuffer.append(respons['256'])
	BLdatabuffer.append(respons['257'])
	BRdatabuffer.append(respons['258'])

	if NORMALIZED:
		for i in range(0,OUTPUTS):
			respons[str(i+255)] = (respons[str(i+255)]-SENSAVG[i])/SENSVAR[i]
		global STARTMAGIC
		if not(STARTMAGIC) and not(STARTRLS) and not(STARTONDERGROND) and experiment.lines[0][0].get_visible():
			print "Start Learning TIME=",time.time()
			global LEARNSTART
			LEARNSTART = time.time()
		if not(check.lines[0][0].get_visible()):
			STARTMAGIC = True

	if POSFB:#don't use, doesn't work, at least it shouldn't work
		datastring = ''.join([", ",str(respons[str(IDS[0])]),", ",str(respons[str(IDS[1])]),", ",str(respons[str(IDS[2])]),", ",str(respons[str(IDS[3])]),", , ",str(respons['255']),", ",str(respons['256']),", ", str(respons['257']),", ",str(respons['258']),", , ",str(sum(FLdatabuffer)/bufsize),", ",str(sum(FRdatabuffer)/bufsize),", ",str(sum(BLdatabuffer)/bufsize),", ",str(sum(BRdatabuffer)/bufsize),"\n"])
#datastring: "FLPOS, FRPOS, BLPOS, BRPOS, , FLSENS, FRSENS, BLSENS, BRSENS\n"
		OUTPUTdatabuffer.append([respons['255'],respons['256'],respons['257'],respons['258'],respons[str(IDS[0])],respons[str(IDS[1])],respons[str(IDS[2])],respons[str(IDS[3])]]) #Building buffer for W matrix with feedbackdata
	else:
		datastring = ''.join([", ",str(respons['255']),", ",str(respons['256']),", ", str(respons['257']),", ",str(respons['258']),", , ",str(sum(FLdatabuffer)/bufsize),", ",str(sum(FRdatabuffer)/bufsize),", ",str(sum(BLdatabuffer)/bufsize),", ",str(sum(BRdatabuffer)/bufsize),"\n"])
		OUTPUTdatabuffer.append(np.concatenate((np.array(lastoutput).flatten(), [respons['255'],respons['256'],respons['257'],respons['258']], [1.0]))) #Building buffer for W matrix with feedbackdata
		global GRONDUPDATECOUNTER
		global UPDATED
		global GROND
		global ONDERGRONDOUTPUT
		global ONDERGRONDINPUT
		if GROND>=0:
			ONDERGRONDOUTPUT.append(np.concatenate((np.array(lastoutput).flatten(), [respons['255'],respons['256'],respons['257'],respons['258']], [1.0])))
			GRONDUPDATECOUNTER = GRONDUPDATECOUNTER + 1
			prediction = np.empty(TYPES)
			prediction.fill(-1.0/(TYPES-1))
			prediction[GROND] = 1.0
			ONDERGRONDINPUT.append(prediction)
			if GRONDUPDATECOUNTER==BSIZE:
				UPDATED[GROND] = 1
				print GROUNDS[GROND], " ready"
				filename = ''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/",GROUNDS[GROND],str(MEMORY),".txt"])
				data = list()
				for i in range(-GRONDUPDATECOUNTER,0):
					data.append(ONDERGRONDOUTPUT[i])
				np.savetxt(filename, np.array(data))
				GROND = -1
					
		else:
			GRONDUPDATECOUNTER = 0
		lastoutput.append([respons['255'],respons['256'],respons['257'],respons['258']])
		
	if LOGGER:
		f = open(''.join(["METINGEN/",CONFIGURATIE,"/LOG/","feedback.txt"]), 'a')
		f.write(datastring)
	return next

#Updates graphs in the plots
#freq = frequency
#plotname = the correct name for the plot to be updated
#other parameters are the spline parameters
def updatePlots(freq,plotname,newx0,newy0,newx1,newy1,newmx0,newmy0,newmx1,newmy1, delay):
	yvals = []
	for element in t:
		yvals.append(hermiteSpline(element,newx0,newy0,newx1,newy1,newmx0,newmy0,newmx1,newmy1, freq, delay))
	plotname.set_ydata(yvals)
	fig.canvas.draw_idle()

#Updates plots and sends data one time if FEEDBACK is False
def update(val):
	updatePlots(sHZ.val,l, 0.0, sFY0.val, sFX1.val, sFY1.val, sFMX0.val, FMY0, sFMX0.val, FMY1, sFD.val)
	updatePlots(sHZ.val,m, 0.0, sFY0.val, sFX1.val, sFY1.val, sFMX0.val, FMY0, sFMX0.val, FMY1, sFD.val)
	updatePlots(sHZ.val,n, 0.0, sBY0.val, sBX1.val, sBY1.val, sBMX0.val, BMY0, sBMX0.val, BMY1, sBD.val)
	updatePlots(sHZ.val,o, 0.0, sBY0.val, sBX1.val, sBY1.val, sBMX0.val, BMY0, sBMX0.val, BMY1, sBD.val)
	if(check.lines[0][0].get_visible() and not(FEEDBACK)):
		print "trying to connect"
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect(("192.168.1.2", 63000))					#select correct IP and port				
		print "connected"
		(respacket,next) = robotreceive(s, '')					#receive one feedback packet
		respacket[0]='' 							#first receive has strange first value
		resp = ''.join(respacket)
		respons = json.loads(resp)
#		print respons
		global IDS
		IDS[0] = respons['255'] #FL
		IDS[1] = respons['256']	#FR
		IDS[2] = respons['257']	#BL
		IDS[3] = respons['258']	#BR
		generateData(s)
		s.close()

#update W matrix [A*B] (W) = (y)*(x+) | [A*B] = [A*size]*[size*B] | (x+) = moore-penrose inverse
#When W is made => generate data with previous sensordata (x)
#new data (y)new = (W)*(x)last timestep | [A*1] = [A*B]*[B*1]
#Linear regression
def updateMagicMatrix():
	global MAGICW
	global INPUTdatabuffer
	global OUTPUTdatabuffer
	moorepenrose = np.linalg.pinv(OUTPUTdatabuffer)
	MAGICW = np.dot(np.array(INPUTdatabuffer).T,moorepenrose.T)
	if LOGGER:
		f = open(''.join(["METINGEN/",CONFIGURATIE,"/EMBODIMENT/","magic.txt"]), 'w')
		f.write(str(MAGICW))

#Update Magic matrix with RLS
#When learntime is over, the matrix is written to a file if LOGGER or SAVEMAGIC is True
#The format of the file is correct to read it at the start of the program (see STARTRLS)
def updateMagicMatrixRLS():
	global MAGICW
	global PRECISION
	global INPUTdatabuffer
	global OUTPUTdatabuffer
	global HALFWAYPOINT
	global MAGICUPDATED
	Lrls = np.dot(PRECISION, np.array([OUTPUTdatabuffer[-1]]).T) / (1.0 + np.dot(np.array([OUTPUTdatabuffer[-1]]), np.dot(PRECISION, np.array([OUTPUTdatabuffer[-1]]).T)))
	PRECISION = PRECISION - (np.dot(PRECISION, np.dot(np.array([OUTPUTdatabuffer[-1]]).T, np.dot(np.array([OUTPUTdatabuffer[-1]]), PRECISION)))) / (1.0 + np.dot(np.array([OUTPUTdatabuffer[-1]]), np.dot(PRECISION, np.array([OUTPUTdatabuffer[-1]]).T)))
	erls =  np.array([INPUTdatabuffer[-1]]).T - np.dot(MAGICW, np.array([OUTPUTdatabuffer[-1]]).T)
	MAGICW = MAGICW + np.dot(erls, Lrls.T)
	T = time.time()
	if T-LEARNSTART>=LEARNTIME/2 and not(MAGICUPDATED) and not(HALFWAYPOINT):
		HALFWAYPOINT = True
		print "Halfway TIME=",time.time()
		if LOGGER:
			f = open(''.join(["METINGEN/",CONFIGURATIE,"/EMBODIMENT/","magic.txt"]), 'w')
			f.write(str(MAGICW))
	elif T-LEARNSTART>=LEARNTIME and not(MAGICUPDATED):
		MAGICUPDATED = True
		HALFWAYPOINT = False
		print "UPDATED TIME=",time.time()
		if LOGGER or SAVEMAGIC:
			f = open(''.join(["METINGEN/",CONFIGURATIE,"/EMBODIMENT/","RLSMagic",str(MEMORY),".txt"]), 'w')
			LOGMAGIC = numpy.array(MAGICW).tolist()
			for rows in LOGMAGIC:
				for ele in rows:
					f.write(''.join([str(ele),", "]))
				f.write("\n")

			f.write(', '.join([', '.join(map(str, SENSAVG)), ', '.join(map(str, SENSVAR)), "\n"]))
	
#GUI THINGS	
#INIT FEEDBACKPLOT
bufsize = 50.0
fig2, ((ax5, ax6), (ax7, ax8)) = plt.subplots(2, 2, sharex='col', sharey='row')	
FLdatabuffer = collections.deque([0.0]*int(bufsize), bufsize)
FRdatabuffer = collections.deque([0.0]*int(bufsize), bufsize)
BLdatabuffer = collections.deque([0.0]*int(bufsize), bufsize)
BRdatabuffer = collections.deque([0.0]*int(bufsize), bufsize)
FLx = np.arange(0.0, bufsize, 1.0)
FLy = np.zeros(bufsize, dtype=np.float)#signalFL
FRy = np.zeros(bufsize, dtype=np.float)#signalFR
BLy = np.zeros(bufsize, dtype=np.float)#signalBL
BRy = np.zeros(bufsize, dtype=np.float)#signalBR
p, = ax5.plot(FLx, FLy, lw=2, color='blue')
q, = ax6.plot(FLx, FRy, lw=2, color='blue')
r, = ax7.plot(FLx, BLy, lw=2, color='blue')
u, = ax8.plot(FLx, BRy, lw=2, color='blue')
ax5.axis([0, bufsize, 0, 2048])
FLAVG = ax5.text(bufsize/3.0, -350.0, ''.join(["FLAVG= ",str(sum(FLy)/bufsize)]))
ax6.axis([0, bufsize, 0, 2048])
FRAVG = ax6.text(bufsize/3.0, -350.0, ''.join(["FRAVG= ",str(sum(FRy)/bufsize)]))
ax7.axis([0, bufsize, 0, 2048])
BLAVG = ax7.text(bufsize/3.0, -350.0, ''.join(["BLAVG= ",str(sum(BLy)/bufsize)]))
ax8.axis([0, bufsize, 0, 2048])
BRAVG = ax8.text(bufsize/3.0, -350.0, ''.join(["BRAVG= ",str(sum(BRy)/bufsize)]))
#END OF FEEDBACKPLOT

#INIT ONDERGRONDPLOT
fig3 = plt.figure()

def startgronddetectie(event):
	global GROND
	index = grondaxes.index(event.inaxes)
	if GROND<0 and not(GRONDMAGICUPDATED):
		if UPDATED[index]:
			UPDATED[:] = 0
			print "Reset learning"
		print "Learning: ", GROUNDS[index]
		GROND = index

grondbuttons = list()
grondaxes = list()
for i in range(0,TYPES):
	grondaxes.append(plt.axes([0.1, 0.8-0.1*i, 0.1, 0.1]))
	grondbuttons.append(Button(grondaxes[i], GROUNDS[i], color='lightgoldenrodyellow', hovercolor='0.975'))
	grondbuttons[i].on_clicked(startgronddetectie)

grondfinishax = plt.axes([0.3, 0.5, 0.1, 0.1])
grondfinish = Button(grondfinishax, 'Finish', color='lightgoldenrodyellow', hovercolor='0.975')
grondnewax = plt.axes([0.3, 0.7, 0.1, 0.1])
grondnew = Button(grondnewax, 'New', color='lightgoldenrodyellow', hovercolor='0.975')


def generategroundmagic(event):
	global GRONDMAGICUPDATED
	if not(GRONDMAGICUPDATED) and sum(UPDATED)==TYPES:
		global ONDERGRONDINPUT
		global ONDERGRONDOUTPUT
		global ONDERGRONDMAGIC
		moorepenrose = np.linalg.pinv(ONDERGRONDOUTPUT)
		ONDERGRONDMAGIC = np.dot(np.array(ONDERGRONDINPUT).T,moorepenrose.T)
		if SAVEMAGIC:
			np.savetxt(''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/","ondergrondmagic",str(MEMORY),"-",str(TYPES),".txt"]),ONDERGRONDMAGIC) #IMPORTANT TO HAVE EXISTING FOLDERS FOR THESE THINGS
			np.savetxt(''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/","ondergrondinput",str(MEMORY),"-",str(TYPES),".txt"]),ONDERGRONDINPUT)
			np.savetxt(''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/","ondergrondoutput",str(MEMORY),"-",str(TYPES),".txt"]),ONDERGRONDOUTPUT)
		f = open(''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/","GROND",str(MEMORY),"-",str(TYPES),".txt"]), 'a')
		f.write(', '.join(GROUNDS))
		f.write("\n")
		GRONDMAGICUPDATED = True

def newgrond(event):
	global NEWGROUND
	NEWGROUND = True



grondfinish.on_clicked(generategroundmagic)
grondnew.on_clicked(newgrond)

#END ONDERGRONDPLOT

#INIT PARAMPLOT
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex='col', sharey='row', figsize=(15,7))
plt.subplots_adjust(left=0.17, bottom=0.45, top=0.97)
t = np.arange(0.0, 1.0, 0.001)
yFLvals = []
yFRvals = []
yBLvals = []
yBRvals = []
for element in t:
	yFLvals.append(hermiteSpline(element, FX0, FY0, FX1, FY1, FMX0, FMY0, FMX1, FMY1, HZ, FD))
	yFRvals.append(hermiteSpline(element, FX0, FY0, FX1, FY1, FMX0, FMY0, FMX1, FMY1, HZ, FD))
	yBLvals.append(hermiteSpline(element, BX0, BY0, BX1, BY1, BMX0, BMY0, BMX1, BMY1, HZ, BD))
	yBRvals.append(hermiteSpline(element, BX0, BY0, BX1, BY1, BMX0, BMY0, BMX1, BMY1, HZ, BD))	
l, = ax1.plot(t,yFLvals, lw=2, color='red')
m, = ax2.plot(t,yFRvals, lw=2, color='red')
n, = ax3.plot(t,yBLvals, lw=2, color='red')
o, = ax4.plot(t,yBRvals, lw=2, color='red')
ax1.axis([0, 1, -3, 3])
ax2.axis([0, 1, -3, 3])
ax3.axis([0, 1, -3, 3])
ax4.axis([0, 1, -3, 3])

rax = plt.axes([0.04, 0.4, 0.1, 0.15])
check = CheckButtons(rax, ('Update',), (False,))

magicax = plt.axes([0.04, 0.55, 0.1, 0.15])
magic = CheckButtons(magicax, ('Magic',), (False,))

experimentax = plt.axes([0.04, 0.70, 0.1, 0.15])
experiment = CheckButtons(experimentax, ('Experiment',), (False,))

def func(label):
	plt.draw()
check.on_clicked(func)

axcolor = 'lightgoldenrodyellow'

#All legs
axfreq = plt.axes([0.20, 0.015, 0.65, 0.015], axisbg=axcolor)
sHZ = Slider(axfreq, 'Freq', 0.00, 3.0, valinit=HZ)

#Front
axFD = plt.axes([0.20, 0.26, 0.65, 0.015], axisbg=axcolor)
axFY0 = plt.axes([0.20, 0.29, 0.65, 0.015], axisbg=axcolor)
axFX1 = plt.axes([0.20, 0.32, 0.65, 0.015], axisbg=axcolor)
axFY1 = plt.axes([0.20, 0.35, 0.65, 0.015], axisbg=axcolor)
axFMX0 = plt.axes([0.20, 0.38, 0.65, 0.015], axisbg=axcolor)

sFD = Slider(axFD, 'FD', -1.0, 1.0, valinit=FD)
sFY0 = Slider(axFY0, 'FY0', -1.5, 1.5, valinit=FY0)
sFX1 = Slider(axFX1, 'FX1', 0.0, 1.0, valinit=FX1)
sFY1 = Slider(axFY1, 'FY1', -1.5, 1.5, valinit=FY1)
sFMX0 = Slider(axFMX0, 'FMX0', 0.0, 0.7, valinit=FMX0)

#Back
axBD = plt.axes([0.20, 0.08, 0.65, 0.015], axisbg=axcolor)
axBY0 = plt.axes([0.20, 0.11, 0.65, 0.015], axisbg=axcolor)
axBX1 = plt.axes([0.20, 0.14, 0.65, 0.015], axisbg=axcolor)
axBY1 = plt.axes([0.20, 0.17, 0.65, 0.015], axisbg=axcolor)
axBMX0 = plt.axes([0.20, 0.20, 0.65, 0.015], axisbg=axcolor)

sBD = Slider(axBD, 'BD', -1.0, 1.0, valinit=BD)
sBY0 = Slider(axBY0, 'BY0', -1.5, 1.5, valinit=BY0)
sBX1 = Slider(axBX1, 'BX1', 0.0, 1.0, valinit=BX1)
sBY1 = Slider(axBY1, 'BY1', -1.5, 1.5, valinit=BY1)
sBMX0 = Slider(axBMX0, 'BMX0', 0.0, 0.7, valinit=BMX0)
	
sHZ.on_changed(update)

sFMX0.on_changed(update)
sFY0.on_changed(update)
sFX1.on_changed(update)
sFY1.on_changed(update)
sFD.on_changed(update)

sBMX0.on_changed(update)
sBY0.on_changed(update)
sBX1.on_changed(update)
sBY1.on_changed(update)
sBD.on_changed(update)

resetax = plt.axes([0.04, 0.35, 0.1, 0.04])
button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')
def reset(event):
	sHZ.reset()
	sFMX0.reset()
	sFY0.reset()
	sFX1.reset()
	sFY1.reset()
	sFD.reset()

	sBMX0.reset()
	sBY0.reset()
	sBX1.reset()
	sBY1.reset()
	sBD.reset()
	magic.lines[0][0].set_visible(False)
	magic.lines[0][1].set_visible(False)
	global STARTMAGIC
	global NORMALIZED
	NORMALIZED = False
	STARTMAGIC = False

button.on_clicked(reset)
#END OF PARAMETERPLOT


#Connects once and repeats sending data and receiving feedback
#This is basically the main program
def feedbackthread(threadname):
	global MAGICUPDATED
	print "trying to connect"
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect(("192.168.1.2", 63000))				#select correct IP and port				
	print "connected"
	(respacket,next) = robotreceive(s, '')					#receive one feedback packet
	respacket[0]='' 										#first receive has strange first value
	resp = ''.join(respacket)
	respons = json.loads(resp)
#	print respons
	global IDS
	IDS[0] = respons['255'] #FL
	IDS[1] = respons['256']	#FR
	IDS[2] = respons['257']	#BL
	IDS[3] = respons['258']	#BR
	while not(THREADSTOP):
		generateData(s)				#generates data dependent on time
		if((magic.lines[0][0].get_visible() or STARTMAGIC) and not(MAGICUPDATED)):
			if(not(RLS)):
				MAGICUPDATED = True
				updateMagicMatrix()
			else:
				updateMagicMatrixRLS()
		elif(not(magic.lines[0][0].get_visible()) and not(STARTMAGIC)):
			global HALFWAYPOINT
			global PRECISION
			global MAGICW
			MAGICUPDATED = False
			HALFWAYPOINT = False
			PRECISION = np.matlib.identity(OUTPUTS*(MEMORY+1)+BIAS)
			MAGICW = [[0.0 for i in range(0,(OUTPUTS*(MEMORY+1)+BIAS))] for j in range(0,INPUTS)]
		next = getRespons(s,next)
	s.close()

#Plot for feedbackdata
#Displays sensor measurements and average
def plotfeedback(threadname):
	while not(THREADSTOP):
		global p
		global q
		global r
		global u
		global FLy
		global FRy
		global BLy
		global BRy
		global FLAVG
		global FRAVG
		global BLAVG
		global BRAVG
		global fig2
		FLy[:] = FLdatabuffer
		FRy[:] = FRdatabuffer
		BLy[:] = BLdatabuffer
		BRy[:] = BRdatabuffer
		p.set_ydata(FLy)
		q.set_ydata(FRy)
		r.set_ydata(BLy)
		u.set_ydata(BRy)
		FLAVG.set_text(''.join(["FLAVG= ",str(sum(FLy)/bufsize)]))
		FRAVG.set_text(''.join(["FRAVG= ",str(sum(FRy)/bufsize)]))
		BLAVG.set_text(''.join(["BLAVG= ",str(sum(BLy)/bufsize)]))
		BRAVG.set_text(''.join(["BRAVG= ",str(sum(BRy)/bufsize)]))
		fig2.canvas.draw_idle()
		time.sleep(0.1)

#This start the robot with a learned W-matrix for RLS
#Correct file format is necessary
if STARTRLS:
	f = open(''.join(["METINGEN/",CONFIGURATIE,"/EMBODIMENT/","RLSMagic",str(MEMORY),".txt"]), 'r')
	readmatrix = f.read().split("\n")[:-1]
	lastline = readmatrix[-1]
	NORMALIZED = True
	normvals = lastline.split(",")
	SENSAVG[:] = np.array(normvals[0:OUTPUTS]).astype(float)
	SENSVAR[:] = np.array(normvals[OUTPUTS:2*OUTPUTS]).astype(float)
	readmatrix = readmatrix[:-1]
	themagic = list()
	for row in readmatrix:
		rlsrow = list()
		row = row.split(",")[:-1]
		for ele in row:
			ele = float(ele)
			rlsrow.append(ele)
		themagic.append(rlsrow)
	MAGICW = np.matrix(themagic)
	magic.lines[0][0].set_visible(True)
	magic.lines[0][1].set_visible(True)
	MAGICUPDATED = True

#The robot start terrain classification with previously learned data
#Folders have to exist!
if STARTONDERGROND:
	for i in range(0,TYPES):
		filename = ''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/",GROUNDS[i],str(MEMORY),".txt"])
		print filename
		arr = np.loadtxt(filename)
		prediction = np.empty(TYPES)
		prediction.fill(-1.0/(TYPES-1))
		prediction[i] = 1.0
		for row in arr:
			ONDERGRONDOUTPUT.append(row)
			ONDERGRONDINPUT.append(prediction)
	moorepenrose = np.linalg.pinv(ONDERGRONDOUTPUT)
	ONDERGRONDMAGIC = np.dot(np.array(ONDERGRONDINPUT).T,moorepenrose.T)
	print "Terrain classification learned TIME=", time.time()
	np.savetxt(''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/","ondergrondmagic",str(MEMORY),"-",str(TYPES),".txt"]),ONDERGRONDMAGIC)
	f = open(''.join(["METINGEN/",CONFIGURATIE,"/GRONDDETECTIE/","GROND",str(MEMORY),"-",str(TYPES),".txt"]), 'a')
	f.write(', '.join(GROUNDS))
	f.write("\n")
	GRONDMAGICUPDATED = True

#This starts the thread that communicates with the Galileo
#PC sends pattern
#Galileo sends feedback
if FEEDBACK:
	#create thread
	fbthread = Thread( target=feedbackthread, args="T" )
	plotthread = Thread( target = plotfeedback, args="S")
	plotthread.start()
	fbthread.start()

plt.show()
#When all windows are closed, all threads are stopped
THREADSTOP = True
	




