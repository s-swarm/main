#!/usr/bin/env python

# light controller for a quadrotor with X configuration
import rospy, time, math, sys
from squaternion import Quaternion
#from pyquaternion import euler2quat, quat2euler, Quaternion
import numpy as np
#import pygame
from pid import PID
import os
import math

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import datetime

from operator import itemgetter

#remove or add the message type
import sensor_msgs
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock 
from geometry_msgs.msg import PointStamped
from mav_msgs.msg import Actuators
from std_msgs.msg import String
from random import random

from rotors_gazebo.msg import FlockMsg

from threading import Thread, Lock

from queue import Queue

lock = Lock()

flockDataQueue = Queue(maxsize=0)

rr_pid = PID(p=0.7, i=1, d=0, imax=10)
pr_pid = PID(p=0.7, i=1, d=0, imax=10)
yr_pid = PID(p=0.7, i=0, d=0.5, imax=10)
rs_pid = PID(p=4.5)
ps_pid = PID(p=4.5)
ys_pid = PID(p=2.0)

STABILIZE 	= 0
ALTHOLD		= 1
POSHOLD		= 2
TAKEOFF		= 3
LANDING		= 4

YUCEL	= 0
FATIH	= 1
WHOIS	= YUCEL

random_targetYaw = [30.513,56.762,-56.078,-166.179,176.573,15.271,84.311,-158.144,92.237,-136.005,15.471,-98.959,-125.219,175.291,74.890,104.770]
random_targetVelocityX = [-14.365,-0.490,-44.473,-22.910,35.548,29.229,12.077,26.415,-43.037,-26.577,47.714,28.351,7.378,-0.241,0.502,23.919]
random_targetVelocityY = [-9.036,34.905,-34.289,-22.267,27.558,-34.847,-11.787,31.464,19.589,-48.276,29.531,44.563,14.294,-31.600,12.504,-15.324]

mode = POSHOLD
roll = 0.0
pitch = 0.0
yaw = 0.0
gyroPitch = 0.0
gyroRoll = 0.0
gyroYaw = 0.0
imuDataReceived = 0
baroDataReceived = 0
baroPressure = 0.0
baroHeight = 0.0
counter = 0

upper_limit = 180.0
lower_limit = -180.0
yaw_target = random_targetYaw[int(sys.argv[2]) % 16]

msgTime = 0.0 # in msecs
Temperature = 25 # in a real scenario measure the temperature and use the current value, from sensor_msgs.msg import Temperature
P0 = 1013.25 # Sea-level pressure in hPa

takeoffAlt	= 400	# get this parameter from terminal
isLanding = 0
isTakeOff = 0

althold_r_pid = PID(p=5.0, i=0.0, d=5.0, imax=10)
althold_s_pid = PID(p=5.0, i=0.0, d=5.0, imax=10)

poshold_pr_pid = PID(p=0.1, i=0.1, d=0.0, imax=5)
poshold_ps_pid = PID(p=0.25, i=0.1, d=0.0, imax=5)
poshold_rr_pid = PID(p=0.1, i=0.1, d=0.0, imax=5)
poshold_rs_pid = PID(p=0.25, i=0.1, d=0.0, imax=5)
currPosX = 0
currPosY = 0
targetPosX = 0
targetPosY = 0
initialVelocity = 50.0 # cm/s
targetVelocityX = random_targetVelocityX[int(sys.argv[2]) % 16]
targetVelocityY = random_targetVelocityY[int(sys.argv[2]) % 16]

positionMatrix = [targetPosX, targetPosY, 1]

target_alt = 200 #random() * (500 - 100) + 100	#prev value 200
print("ardrone"+str(sys.argv[2])+" target_alt = "+str(target_alt))

prev_posX = 0
prev_posY = 0
velPosX = 0
velPosY = 0
currentThr = 655

baroMsgTime = 0.0 # in msecs
ascent_descent_rate = 0
prev_baro_height = 0
prev_baro_msg_time = 0


althold_rate_output = 0.0
althold_stab_output = 0.0

rcpit = 0.0
rcroll = 0.0

insideImuCallback = 0
insidePosCallback = 0
insideFlockPosCallback = 0

positionControlIterator = 0

startInitWP = 0
currTime = 0

#definition of global variables for finding neighbors calcs.
N = int(sys.argv[1])		#number of UAVs
myID = int(sys.argv[2])
r = int(sys.argv[3])*100	#radius for metric distance (convert data received in meters to cm.)

logFileName = open(datetime.datetime.now().strftime("dataLogs/%Y_%m_%d_%H_%M_%S.txt"), "w+")	#open file and log flock_info to /flock_logs/file.txt

cmdStartFlag = 0
cmdStopFlag = 0
cmdFlockingFlag = 0
cmdVelocityFlag = 0

flockUavPoints = []		#information (pos,vel,yaw) about each Uav in flock
flockUavDistances = []	#distances of this controller (myID) to each other
detectedUavs = []		#detected drones according to r.
countDetectedUavs = 0	#number of detected drones according to r.

avgPosX = 0
avgPosY = 0
avgHeight = 0
avgVelPosX = 0
avgVelPosY = 0
avgAscentDescentRate = 0
avgYaw = 0

flockVelocity = [50.0, 50.0, 0.0]

separationUavs = []		#closest uavs 
countSeperationUavs = 0	#number of detected drones according to r.
separationLimit = 200	#seperate Uavs according to this range limit. (cm)

seperationPosX = 0
seperationPosY = 0
seperationHeight = 0
seperationVelPosX = 0
seperationVelPosY = 0
seperationAscentDescentRate = 0
seperationYaw = 0

uavName = "ardrone" + str(myID)
motorPublisher = rospy.Publisher(uavName+'/command/motor_speed', Actuators, queue_size=10)

def wrap_180(x):
	if x < -180.0:
		return x + 360
	elif x > 180:
		return x - 360
	else:
		return x


def clock_callback(msg):	
	global startInitWP, currTime

	clockTimeMillis = msg.clock.secs*1000.0 + msg.clock.nsecs/1000000.0
	
	if (startInitWP == 0):
		#currTime = msg.clock.secs
		currTime = clockTimeMillis
		startInitWP = 1
	else:
		if ((clockTimeMillis - currTime) >= 50):
			currTime = clockTimeMillis			
			sendFlockMessage()
			#print(str(uavName) + " " + str(currTime/1000.0))	
	return
	
def altitude_controller(baroHeightLocal, baroMsgTimeLocal, ascent_descent_rateLocal):
	global target_alt, althold_rate_output, althold_stab_output
	
	if cmdFlockingFlag == 0:
		althold_stab_output = max(min(althold_s_pid.get_pid(target_alt - baroHeightLocal, 1, baroMsgTimeLocal), 25), -25)
		althold_rate_output = max(min(althold_r_pid.get_pid(althold_stab_output - ascent_descent_rateLocal*100.0, 1, baroMsgTimeLocal), 25), -25)
	else:
		althold_stab_output = max(min(althold_s_pid.get_pid(target_alt - baroHeightLocal, 1, baroMsgTimeLocal), 25), -25)
		althold_rate_output = max(min(althold_r_pid.get_pid(althold_stab_output - ascent_descent_rateLocal*100.0, 1, baroMsgTimeLocal), 25), -25)
	
def attitude_controller(gyroPitch, gyroRoll, gyroYaw, roll, pitch, yaw, msgTime):

	global currentThr, motorPublisher, althold_rate_output, rcpit, rcroll, cmdStopFlag, logFileName

	pitch_stab_output = max(min(ps_pid.get_pid(rcpit - pitch, 1, msgTime), 25), -25)
	roll_stab_output  = max(min(rs_pid.get_pid(rcroll - roll, 1, msgTime), 25), -25)
	yaw_stab_output   = max(min(ys_pid.get_pid(wrap_180(yaw_target - yaw), 1, msgTime), 40), -40)
		
	## Rate PIDS
	pitch_output =  max(min(pr_pid.get_pid(pitch_stab_output - gyroPitch, 1, msgTime), 50), -50)
	roll_output  =  max(min(rr_pid.get_pid(roll_stab_output - gyroRoll, 1, msgTime), 50), -50)
	yaw_output   =  max(min(yr_pid.get_pid(yaw_stab_output - gyroYaw, 1, msgTime), 50), -50)

	calc_angle_values = math.cos((abs(pitch)/180.0)*math.pi) * math.cos((abs(roll)/180.0)*math.pi)
	motorVelFL = min(currentThr + roll_output - pitch_output + yaw_output + althold_rate_output, 2000) / calc_angle_values
	motorVelBL = min(currentThr + roll_output + pitch_output - yaw_output + althold_rate_output, 2000) / calc_angle_values
	motorVelFR = min(currentThr - roll_output - pitch_output - yaw_output + althold_rate_output, 2000) / calc_angle_values
	motorVelBR = min(currentThr - roll_output + pitch_output + yaw_output + althold_rate_output, 2000) / calc_angle_values

	msg = Actuators()
	if cmdStopFlag == 0:
		msg.angular_velocities = [motorVelFR, motorVelBL, motorVelFL, motorVelBR]

		#fill log file
		logFileName.write(str(round(baroMsgTime/1000))+","+uavName+","+str(round(currPosX, 3))+","+str(round(currPosY, 3))+","+str(round(baroHeight, 3))+","+
							str(round(velPosX, 3))+","+str(round(velPosY, 3))+","+str(round(ascent_descent_rate, 3))+","+str(round(yaw, 3))+"\n")
		
	else:
		msg.angular_velocities = [0, 0, 0, 0]
		cmdStopFlag = cmdStopFlag + 1
		if cmdStopFlag == 100:
			#sys.exit()
			print(uavName+" controller killed.. PID = "+str(os.getpid()))
			os.system("kill %d" % os.getpid())
	motorPublisher.publish(msg)
	

def imu_callback(msg):
	global gyroPitch, gyroRoll, gyroYaw, imuDataReceived, roll, pitch, yaw, msgTime, insideImuCallback

	if (insideImuCallback == 0):
		insideImuCallback = 1
	else:
		print("insideImuCallback")
		return

	msgTime = msg.header.stamp.secs*1000.0 + msg.header.stamp.nsecs/1000000.0
	m = msg.orientation
	q = Quaternion(m.w,m.x,m.y,m.z)
	[roll, pitch, yaw] = q.to_euler(degrees=True)
	
	g = msg.angular_velocity
	[gyroRoll,gyroPitch,gyroYaw] = np.degrees([g.x, g.y, g.z])
	
	attitude_controller(gyroPitch, gyroRoll, gyroYaw, roll, pitch, yaw, msgTime)
	insideImuCallback = 0


def position_controller(currPosXLocal, currPosYLocal, velPosXLocal, velPosYLocal, baroMsgTimeLocal):

	global rcpit, rcroll, yaw, positionMatrix, avgPosX, avgPosY, avgVelPosX, avgVelPosY

	if(yaw < 0):
		posHoldYaw = yaw + 360.0
	else:
		posHoldYaw = yaw
	posHoldYaw = (posHoldYaw + 270.0) % 360.0
	
	cosValue = math.cos(posHoldYaw * math.pi /180.0)
	sinValue = math.sin(posHoldYaw * math.pi /180.0)
		
	if cmdFlockingFlag == 1:
		yaw_target = avgYaw
		yaw_target = (yaw_target + seperationYaw) / 2
		
		positionMatrix = [avgPosX, avgPosY, 1]
		# we should transform calculated avg velocities from global to local
		avgVelMatrix = [avgVelPosX, avgVelPosY, 1] 		#average velocities in global coordinate frame (from flock_data)
		transformationMatrix = [[cosValue, -sinValue, 0], [sinValue, cosValue, 0], [0, 0, 1]] # transformation from local to global
		invTransformationMatrix = np.linalg.inv(transformationMatrix) # take inverse of above transformation and get global to local transformation
		avgVelocityResultMatrix = np.matmul(invTransformationMatrix, avgVelMatrix) # apply global to local transformation and get velocities in local coordinate frame
		avgVelPosX = avgVelocityResultMatrix[0]
		avgVelPosY = avgVelocityResultMatrix[1]
		
	#prepare transformation matrix for converting target position corrdinates from global to local
	transformationMatrix = [[cosValue, -sinValue, currPosXLocal], [sinValue, cosValue, currPosYLocal], [0, 0, 1]] # this is from local to global
	invTransformationMatrix = np.linalg.inv(transformationMatrix) # we have take its inverse to get global to local transformation
	resultMatrix = np.matmul(invTransformationMatrix, positionMatrix) # apply transformation to current target coordinated and get their corresponding coord. in local coord. frame
	targetPosX = resultMatrix[0]
	targetPosY = resultMatrix[1]

	# we should transform velocities from global to local
	velocityMatrix = [velPosXLocal, velPosYLocal, 1] # current velocities in global coordinate frame
	transformationMatrix = [[cosValue, -sinValue, 0], [sinValue, cosValue, 0], [0, 0, 1]] # transformation from local to global
	invTransformationMatrix = np.linalg.inv(transformationMatrix) # take inverse of above transformation and get global to local transformation
	velocityResultMatrix = np.matmul(invTransformationMatrix, velocityMatrix) # apply global to local transformation and get velocities in local coordinate frame
	velPosXLocal = velocityResultMatrix[0]
	velPosYLocal = velocityResultMatrix[1]

	poshold_p_stab_output = max(min(poshold_ps_pid.get_pid(targetPosY, 1, baroMsgTimeLocal), 250), -250)
	poshold_r_stab_output = max(min(poshold_rs_pid.get_pid(targetPosX, 1, baroMsgTimeLocal), 250), -250)
	
	#flock_behaviour_velocity_matching(x,y)
	if cmdFlockingFlag == 1:
		poshold_p_stab_output = poshold_p_stab_output + avgVelPosX + flockVelocity[1] 		#flockVelocity currently 0
		poshold_r_stab_output = poshold_r_stab_output + avgVelPosY + flockVelocity[0]		#flockVelocity currently 0
	else: 
		poshold_p_stab_output = 0
		poshold_r_stab_output = 0	
		
	poshold_p_output = max(min(poshold_pr_pid.get_pid(poshold_p_stab_output - velPosYLocal, 1, baroMsgTimeLocal), 25), -25)
	poshold_r_output = max(min(poshold_rr_pid.get_pid(poshold_r_stab_output - velPosXLocal, 1, baroMsgTimeLocal), 25), -25)

	rcpit = poshold_p_output * 1.0		
	rcroll = poshold_r_output * 1.0				
		
def velocity_controller(currPosXLocal, currPosYLocal, velPosXLocal, velPosYLocal, baroMsgTimeLocal):

	global rcpit, rcroll, yaw, positionMatrix, avgPosX, avgPosY, avgVelPosX, avgVelPosY

	if(yaw < 0):
		posHoldYaw = yaw + 360.0
	else:
		posHoldYaw = yaw
	posHoldYaw = (posHoldYaw + 270.0) % 360.0
	
	cosValue = math.cos(posHoldYaw * math.pi /180.0)
	sinValue = math.sin(posHoldYaw * math.pi /180.0)

	velocityMatrix = [velPosXLocal, velPosYLocal, 1] # current velocities in global coordinate frame
	transformationMatrix = [[cosValue, -sinValue, 0], [sinValue, cosValue, 0], [0, 0, 1]] # transformation from local to global
	invTransformationMatrix = np.linalg.inv(transformationMatrix) # take inverse of above transformation and get global to local transformation
	velocityResultMatrix = np.matmul(invTransformationMatrix, velocityMatrix) # apply global to local transformation and get velocities in local coordinate frame
	velPosXLocal = velocityResultMatrix[0]
	velPosYLocal = velocityResultMatrix[1]

	targetVelocityMatrix = [targetVelocityX, targetVelocityY, 1] # target velocities in global coordinate frame
	targetVelocityResultMatrix = np.matmul(invTransformationMatrix, targetVelocityMatrix) # apply global to local transformation and get velocities in local coordinate frame
	targetVelPosXLocal = targetVelocityResultMatrix[0]
	targetVelPosYLocal = targetVelocityResultMatrix[1]
	
	
	poshold_p_output = max(min(poshold_pr_pid.get_pid(targetVelPosYLocal - velPosYLocal, 1, baroMsgTimeLocal), 25), -25)
	poshold_r_output = max(min(poshold_rr_pid.get_pid(targetVelPosXLocal - velPosXLocal, 1, baroMsgTimeLocal), 25), -25)

	rcpit = poshold_p_output * 1.0		
	rcroll = poshold_r_output * 1.0				
		

def transform_flock_data(data):
	global flockUavPoints
	flockUavPoints.clear()
	data = str(data)[7:-1]	#clean data. [data: "..."]
	uavList = data.split(";")

	counter = 0
	while(counter < len(uavList)-1):
		new_point = [0.0] * 8
			
		uavInfo = uavList[counter].split(":")
		new_point[0] = int(str(uavInfo[0])[7:])


		temp = uavInfo[1].split(",")
		new_point[1] = float(temp[0])	#posX
		new_point[2] = float(temp[1])	#posY
		new_point[3] = float(temp[2])	#baroHeight
		new_point[4] = float(temp[3])	#velX
		new_point[5] = float(temp[4])	#velY
		new_point[6] = float(temp[5])	#ascDescRate
		new_point[7] = float(temp[6])	#yaw

		counter = counter + 1

		flockUavPoints.append(new_point)
	flockUavPoints =sorted(flockUavPoints, key=itemgetter(0))


def calc_dist_points(p1, p2):
	return math.sqrt(((p1[1]-p2[1])**2)+((p1[2]-p2[2])**2)+((p1[3]-p2[3])**2))	

		
def get_metric_distance():
	global flockUavDistances
	
	flockUavDistances = [0.0 for x in range(N)]
	
	for i in range(N):
			flockUavDistances[i] = calc_dist_points(flockUavPoints[myID], flockUavPoints[i])	

def get_nearest_neighbors():
	global detectedUavs, countDetectedUavs, separationUavs, countSeperationUavs
	
	detectedUavs = [0.0 for x in range(N)]
	separationUavs = [0.0 for x in range(N)]
	
	countDetectedUavs = 0
	countSeperationUavs = 0
	for i in range(N):
		newDist = flockUavDistances[i]		#get distance between this UAV and i th drone
		if newDist < r and newDist != 0:	
			detectedUavs[i] = flockUavDistances[i]		
			countDetectedUavs = countDetectedUavs + 1 
			#print("+ "+str(i)+". added with dist = "+str(flockUavDistances[myID][i]))

			if newDist < seperationLimit:
				separationUavs[i] = flockUavDistances[i]
				countSeperationUavs = countSeperationUavs + 1

def	avg_neighbors_data():
	global avgPosX, avgPosY, avgHeight, avgVelPosX, avgVelPosY, avgAscentDescentRate, avgYaw, yaw_target, seperationPosX, seperationPosY, seperationHeight, seperationVelPosX, seperationVelPos, seperationAscentDescentRate,	seperationYaw
	
	tempAvgInfos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]			#posX, posY, height, velX, velY, rate, yaw
	tempSeperationInfos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]	#posX, posY, height, velX, velY, rate, yaw
	
	for i in range(N):
		if detectedUavs[i] == 0:			#if i th index is equal to zero, this uav is not a neighbor.
			continue	
		else:

			tempAvgInfos[0] = tempAvgInfos[0] + float(flockUavPoints[i][1])
			tempAvgInfos[1] = tempAvgInfos[1] + float(flockUavPoints[i][2])
			tempAvgInfos[2] = tempAvgInfos[2] + float(flockUavPoints[i][3])
			tempAvgInfos[3] = tempAvgInfos[3] + float(flockUavPoints[i][4])
			tempAvgInfos[4] = tempAvgInfos[4] + float(flockUavPoints[i][5])
			tempAvgInfos[5] = tempAvgInfos[5] + float(flockUavPoints[i][6])
			tempAvgInfos[6] = tempAvgInfos[6] + float(flockUavPoints[i][7])

			if float(flockUavPoints[i][7]) < 0: # eger son eklenen aci 0 ile -180 arasindaysa 360 ekleyerek eklenen son aciyi 0 ile 360 arasina cevirmis olalim
				tempAvgInfos[6] = tempAvgInfos[6] + 360 
				
			if separationUavs[i] != 0:				#if i th index is not equal to zero, this uav is so close!
				tempSeperationInfos[0] = tempSeperationInfos[0] + float(flockUavPoints[i][1])
				tempSeperationInfos[1] = tempSeperationInfos[1] + float(flockUavPoints[i][2])
				tempSeperationInfos[2] = tempSeperationInfos[2] + float(flockUavPoints[i][3])
				tempSeperationInfos[3] = tempSeperationInfos[3] + float(flockUavPoints[i][4])
				tempSeperationInfos[4] = tempSeperationInfos[4] + float(flockUavPoints[i][5])
				tempSeperationInfos[5] = tempSeperationInfos[5] + float(flockUavPoints[i][6])
				tempSeperationInfos[6] = tempSeperationInfos[6] + float(flockUavPoints[i][7])
				
				if float(flockUavPoints[i][7]) < 0: # eger son eklenen aci 0 ile -180 arasindaysa 360 ekleyerek eklenen son aciyi 0 ile 360 arasina cevirmis olalim
					tempSeperationInfos[6] = tempSeperationInfos[6] + 360 
			
	if countDetectedUavs == 0:
		print("No neighboring drones detected within a "+ str(r) +" cm radius. ")

	else:
		avgPosX = tempAvgInfos[0]/countDetectedUavs
		avgPosY = tempAvgInfos[1]/countDetectedUavs
		avgHeight = tempAvgInfos[2]/countDetectedUavs
		avgVelPosX = tempAvgInfos[3]/countDetectedUavs
		avgVelPosY = tempAvgInfos[4]/countDetectedUavs
		avgAscentDescentRate = tempAvgInfos[5]/countDetectedUavs
		avgYaw = tempAvgInfos[6]/countDetectedUavs

	if countSeperationUavs == 0:
		print("No close neighbor detected within a "+ str(seperationLimit) +" cm radius for "+uavName)
		#continue
	else:
		seperationPosX = tempSeperationInfos[0]/countSeperationUavs
		seperationPosY = tempSeperationInfos[1]/countSeperationUavs
		seperationHeight = tempSeperationInfos[2]/countSeperationUavs
		seperationVelPosX = tempSeperationInfos[3]/countSeperationUavs
		seperationVelPosY = tempSeperationInfos[4]/countSeperationUavs
		seperationAscentDescentRate = tempSeperationInfos[5]/countSeperationUavs
		seperationYaw = tempSeperationInfos[6]/countSeperationUavs


def flock_msg_callback(data):	
	global flockDataQueue, insideFlockPosCallback

	if (insideFlockPosCallback == 0):
		insideFlockPosCallback = 1
	else:
		print("insideFlockPosCallback")

	for i in range(N):
		temp = "ardrone"+str(i)
		if temp not in str(data): 
			return

	if cmdFlockingFlag == 1: # IF WE ARE FLOCKING!!!
		transform_flock_data(data)
		get_metric_distance()
		lock.acquire()
		if len(flockUavPoints) == 0:
			print(str(myID) + ": flockUavPoints empty! ")
		flockTuple = (flockUavDistances,flockUavPoints)
		flockDataQueue.put(flockTuple)

		(uavDistances,uavPoints) = flockDataQueue.get()
		if len(uavPoints) == 0:
			print(str(myID) + ": flockUavPoints empty after first put-get! len of flockUavPoints:" + str(len(flockUavPoints)))
		flockDataQueue.put(flockTuple)
		lock.release()
		
	insideFlockPosCallback = 0

def command_msg_callback(data):
	global cmdStartFlag, cmdStopFlag, cmdFlockingFlag, cmdVelocityFlag, logFileName
	data = str(data)[7:-1]	#clean data. [data: "..."]
	if data == "start":
		cmdStartFlag = 1

	if data == "flock":
		cmdVelocityFlag = 0
		cmdFlockingFlag = 1
	
		logFileName.write("currTime, uavName, currPosX, currPosY, baroHeight, velPosX, velPosY, ascent_descent_rate, yaw\n")
	
	if data == "velocity":
		cmdVelocityFlag = 1
		print("ardrone:"+str(myID)+", targetVelocityX:" + str(targetVelocityX) + ", targetVelocityY: " + str(targetVelocityY))

	if data == "stop":
		cmdStopFlag = 1

		logFileName.close()
	
def sendFlockMessage():
	global myID, currPosX, currPosY, baroHeight, velPosX, velPosY, ascent_descent_rate, yaw
	
	pub = rospy.Publisher('talking_topic_'+uavName, FlockMsg, queue_size=10)
	msgPubInfo = FlockMsg()
	
	msgPubInfo.ID = uavName
	msgPubInfo.x = currPosX
	msgPubInfo.y = currPosY
	msgPubInfo.z = baroHeight
	msgPubInfo.vx = velPosX
	msgPubInfo.vy = velPosY
	msgPubInfo.vz = ascent_descent_rate
	msgPubInfo.yaw = yaw
		
	pub.publish(msgPubInfo)	

def calculate_logistic_function(x, slope=2, desired_distance=3):
	value = (2 / (1 + math.exp(slope * (x - desired_distance)))) - 1
	return value 

def flocking_alignment(uavDistances,uavPoints):

	averageVelocityX = 0.0
	averageVelocityY = 0.0
	averageYawValue = 0.0
	numOfAlignmentNeighbors = 0

	#print(str(myID) + ": " + str(flockUavDistances) + ", " + str(flockUavPoints))
	#print(str(myID) + ": " + str(uavDistances) + ", " + str(uavPoints))

	for i in range(N):
		if uavDistances[i] < r and uavDistances[i] > 0.01 :

			try:
				averageVelocityX = averageVelocityX + float(uavPoints[i][4])
				averageVelocityY = averageVelocityY + float(uavPoints[i][5])
				averageYawValue = averageYawValue + float(uavPoints[i][7])
				if float(uavPoints[i][7]) < 0: 
					averageYawValue = averageYawValue + 360 
					
				numOfAlignmentNeighbors = numOfAlignmentNeighbors + 1
			except IndexError:
				print("IndexError exception: " + str(myID) + " len: " + str(len(uavPoints)))
				
	if numOfAlignmentNeighbors == 0:
		return [math.nan, math.nan, math.nan]
	else:
		averageVelocityX = averageVelocityX / float(numOfAlignmentNeighbors)
		averageVelocityY = averageVelocityY / float(numOfAlignmentNeighbors)
		averageYawValue = averageYawValue / float(numOfAlignmentNeighbors)

	return [averageVelocityX, averageVelocityY, averageYawValue]	


def cohesion_behavior(uavDistances,uavPoints):

	averagePosX = 0.0
	averagePosY = 0.0
	numOfCohesionNeighbors = 0

	for i in range(N):
		if uavDistances[i] < r and uavDistances[i] > 0.01 :

			try:
				averagePosX = averagePosX + float(uavPoints[i][1])
				averagePosY = averagePosY + float(uavPoints[i][2])
				numOfCohesionNeighbors = numOfCohesionNeighbors + 1
			except IndexError:
				print("IndexError exception in cohesion: " + str(myID) + " len: " + str(len(uavPoints)))
				
	if numOfCohesionNeighbors == 0:
		return [math.nan, math.nan]
	else:
		averagePosX = averagePosX / float(numOfCohesionNeighbors)
		averagePosY = averagePosY / float(numOfCohesionNeighbors)
	return [averagePosX, averagePosY]	


def separation_behavior(uavDistances,uavPoints):
	averagePosX = 0.0
	averagePosY = 0.0
	numOfSeparationNeighbors = 0

	for i in range(N):
		if uavDistances[i] < 200 and uavDistances[i] > 0.01 :

			try:
				averagePosX = averagePosX + float(uavPoints[i][1])
				averagePosY = averagePosY + float(uavPoints[i][2])
				numOfSeparationNeighbors = numOfSeparationNeighbors + 1
			except IndexError:
				print("IndexError exception in cohesion: " + str(myID) + " len: " + str(len(uavPoints)))

	if numOfSeparationNeighbors == 0:
		return [math.nan, math.nan]
	else:
		averagePosX = averagePosX / float(numOfSeparationNeighbors)
		averagePosY = averagePosY / float(numOfSeparationNeighbors)

	return [averagePosX, averagePosY]

def	flock_controller2D(currPosXLocal, currPosYLocal, velPosXLocal, velPosYLocal, baroMsgTimeLocal):
	global avgPosX, avgPosY, avgHeight, avgVelPosX, avgVelPosY, avgAscentDescentRate, avgYaw, yaw_target 
	global seperationPosX, seperationPosY, seperationHeight, seperationVelPosX, seperationVelPos, seperationAscentDescentRate,	seperationYaw
	#global 
	global rcpit, rcroll, yaw, positionMatrix, avgPosX, avgPosY, avgVelPosX, avgVelPosY
	global althold_stab_output
	global flockDataQueue

	if flockDataQueue.qsize() > 0:
		lock.acquire()
		(uavDistances,uavPoints) = flockDataQueue.get()
		if len(uavPoints) == 0:
			print(str(myID) + ": in flock controller 2D uavPoints empty")
			lock.release()
			return

		lock.release()
	else:
		return


	[averageCohesionPosX, averageCohesionPosY] = cohesion_behavior(uavDistances,uavPoints) # velocity_matching behaviour
	if math.isnan(averageCohesionPosX) and math.isnan(averageCohesionPosY):
		#return
		cohesionPosX = 0.0
		cohesionPosY = 0.0
	else:
		cohesionPosX = averageCohesionPosX - currPosXLocal
		cohesionPosY = averageCohesionPosY - currPosYLocal

	[averageSeparationPosX, averageSeparationPosY] = separation_behavior(uavDistances,uavPoints) # velocity_matching behaviour

	if math.isnan(averageSeparationPosX) and math.isnan(averageSeparationPosY):
		separationPosX = 0.0
		separationPosY = 0.0
	else:
		separationPosX = -(averageSeparationPosX - currPosXLocal)
		separationPosY = -(averageSeparationPosY - currPosYLocal)

	[averageVelocityX, averageVelocityY, averageYawValue] = flocking_alignment(uavDistances,uavPoints) # velocity_matching behaviour
	if math.isnan(averageVelocityX) and math.isnan(averageVelocityX):
		averageVelocityX = 0.0
		averageVelocityY = 0.0
		#return
	else: 
		yaw_target = averageYawValue

	flockVelPosX = max(min(averageVelocityX + 0.3*cohesionPosX + 1.2*separationPosX,100.0)+flockVelocity[0],-100.0)
	flockVelPosY = max(min(averageVelocityY	+ 0.3*cohesionPosY + 1.2*separationPosY,100.0)+flockVelocity[1],-100.0)
		
	if(yaw < 0):
		posHoldYaw = yaw + 360.0
	else:
		posHoldYaw = yaw
	posHoldYaw = (posHoldYaw + 270.0) % 360.0
	
	cosValue = math.cos(posHoldYaw * math.pi /180.0)
	sinValue = math.sin(posHoldYaw * math.pi /180.0)
		
	# we should transform calculated flock velocities from global to local
	flockVelMatrix = [flockVelPosX, flockVelPosY, 1] 		#average velocities in global coordinate frame (from flock_data)
	transformationMatrix = [[cosValue, -sinValue, 0], [sinValue, cosValue, 0], [0, 0, 1]] # transformation from local to global
	invTransformationMatrix = np.linalg.inv(transformationMatrix) # take inverse of above transformation and get global to local transformation
	avgVelocityResultMatrix = np.matmul(invTransformationMatrix, flockVelMatrix) # apply global to local transformation and get velocities in local coordinate frame
	flockVelPosX = avgVelocityResultMatrix[0]
	flockVelPosY = avgVelocityResultMatrix[1]
		
	# we should transform velocities from global to local
	velocityMatrix = [velPosXLocal, velPosYLocal, 1] # current velocities in global coordinate frame
	#velocityMatrix = [velPosXLocal*1000.0, velPosYLocal*1000.0, 1]
	transformationMatrix = [[cosValue, -sinValue, 0], [sinValue, cosValue, 0], [0, 0, 1]] # transformation from local to global
	invTransformationMatrix = np.linalg.inv(transformationMatrix) # take inverse of above transformation and get global to local transformation
	velocityResultMatrix = np.matmul(invTransformationMatrix, velocityMatrix) # apply global to local transformation and get velocities in local coordinate frame
	velPosXLocal = velocityResultMatrix[0]
	velPosYLocal = velocityResultMatrix[1]

	poshold_p_output = max(min(poshold_pr_pid.get_pid(flockVelPosY - velPosYLocal, 1, baroMsgTimeLocal), 25), -25)
	poshold_r_output = max(min(poshold_rr_pid.get_pid(flockVelPosX - velPosXLocal, 1, baroMsgTimeLocal), 25), -25)

	rcpit = poshold_p_output * 1.0		
	rcroll = poshold_r_output * 1.0

	return

def position_callback(msg):
	global baroDataReceived, baroPressure, Temperature, Pval, baroHeight, P0, baroMsgTime, ascent_descent_rate, currPosX, currPosY, velPosX, velPosY, prev_posX, prev_posY, insidePosCallback
	global positionControlIterator
	
	positionControlIterator = positionControlIterator + 1
	if(positionControlIterator < 8):
		return
	else:
		positionControlIterator = 0

	if (insidePosCallback == 0):
		insidePosCallback = 1
	else:
		print("insidePosCallback")
		return

	prev_baro_height = baroHeight
	prev_baro_msg_time = baroMsgTime
	baroMsgTime = msg.header.stamp.secs*1000.0 + msg.header.stamp.nsecs/1000000.0
	
	baroHeight = msg.point.z*100.0 #+ s[0]
	baroDataReceived = 1
	ascent_descent_rate = (baroHeight - prev_baro_height)/(baroMsgTime - prev_baro_msg_time)
	
	prev_posX = currPosX
	prev_posY = currPosY
	currPosX = msg.point.x*100 #+ s[1]
	velPosX = (currPosX - prev_posX)/(baroMsgTime - prev_baro_msg_time)*1000.0 # velocity for x-axis in global coordinate frame [cm/s]

	currPosY = msg.point.y*100 #+ s[2]
	velPosY = (currPosY - prev_posY)/(baroMsgTime - prev_baro_msg_time)*1000.0 # velocity for y-axis in global coordinate frame [cm/s]

	if cmdVelocityFlag == 1:
		velocity_controller(currPosX, currPosY, velPosX, velPosY, baroMsgTime)
	elif cmdFlockingFlag == 0:
		position_controller(currPosX, currPosY, velPosX, velPosY, baroMsgTime)
	else:
		flock_controller2D(currPosX, currPosY, velPosX, velPosY, baroMsgTime)
	altitude_controller(baroHeight, baroMsgTime, ascent_descent_rate)
	insidePosCallback = 0
	
if __name__=='__main__':
	rospy.init_node(uavName+'imu_subscriber',anonymous=True)

	subImu=rospy.Subscriber(uavName+'/imu', Imu, imu_callback)
	subClock=rospy.Subscriber('/clock', Clock, clock_callback)
	subPosition=rospy.Subscriber('/'+uavName+'/odometry_sensor1/position', PointStamped, position_callback)

	rospy.Subscriber('sensor_node', String, flock_msg_callback)
	rospy.Subscriber('command_node', String, command_msg_callback)
	
	rospy.spin()
