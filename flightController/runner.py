#python flockParam.py [uav_number] [radius(m)]

import rospy, time, math, sys
import numpy as np
from matplotlib import pyplot as plt
import pygame
from pid import PID
import os
import math
import psutil	#terminate process lib


from std_msgs.msg  import String
from mav_msgs.msg import Actuators

from pynput import keyboard


numberOfUAVs = int(sys.argv[1])
r = int(sys.argv[2])

startFlag = 0
flockFlag = 0

cmd_pub = rospy.Publisher('/command_node', String, queue_size=10)
command_data = String('stop')

		
def init_UAVs():	
	cmd = ""
	for i in range(0, numberOfUAVs):
		cmd = "python controller.py " +str(numberOfUAVs) + " " + str(i) + " " + str(r) +" & "
		os.system(cmd)
		time.sleep(2)

def run_cmd(cmd):
	os.system(cmd)	
	print(cmd)
			

def on_press(key):
	try:
		global startFlag, flockFlag, cmd_pub, command_data
		
		print('alphanumeric key {0} pressed'.format(key.char))
		
		if key.char == 's':
			if startFlag > 0:
				print("Already started.. Press 'q' to quit.")
			else:
				startFlag = 1
				cmd_pub.publish('start')				
				print("s pressed, starting..")

				run_cmd("rosservice call /gazebo/reset_world \"" + "{}\"")
				run_cmd("rosservice call gazebo/unpause_physics")
				init_UAVs()
				
		if key.char == 'f':
			if startFlag > 0:
				if flockFlag == 0:
					flockFlag = 1
					cmd_pub.publish('flock')
					print("f pressed, starting flocking..")
				else:
					print("Already started flocking.. Press 'q' to quit.")
			elif startFlag == 0:
				print("f pressed, you should start first by pressing 's'..")
		
		if key.char == 'v':
			if startFlag > 0:
				if flockFlag == 0:
					cmd_pub.publish('velocity')
					print("v pressed, starting random velocity..")
				else:
					print("Already started flocking.. Press 'q' to quit.")
			elif startFlag == 0:
				print("f pressed, you should start first by pressing 's'..")
		
		if key.char == 'q':
			print("q pressed, exiting...")

			cmd_pub.publish('stop')
			
			time.sleep(1)
			sys.exit()
		
	except AttributeError:
		print('special key {0} pressed'.format(key))

if __name__=='__main__':
	rospy.init_node('flock_coordinator',anonymous=True)
	print("Keyboard listening.. Press 's' to start, 'q' to quit:")
	
	# Collect events until released
	with keyboard.Listener(on_press=on_press) as listener:
		listener.join()
		
