#python uavMsgSubscriber.py [uav_number]
#!/usr/bin/env

import rospy, sys
from rotors_gazebo.msg import FlockMsg
from std_msgs.msg  import String
import time
import os

#published message format
#rostopic pub /sensor_node std_msgs/String "uav0:1,1,1,1,1,1,111;uav1:1,1,1,1,1,1,111;uav2:1,1,1,1,1,1,111;uav3:1,1,1,1,1,1,111"

numberOfUAVs = int(sys.argv[1])
subscriber_msg_flag = 0
pub = rospy.Publisher('sensor_node', String, queue_size=10)
flock_data = ""

def findNeighbors():
	global flock_data

	pub.publish(flock_data)
	print("..flock_data published..")
	print(flock_data)
	flock_data = ""

def checkAllDataReceived():
#check flock_data; if got data from all UAVs, then call findNeighbors function.
	counter = 0
	for i in range(0, numberOfUAVs):
		if "ardrone"+str(i)+":" in str(flock_data):
			counter = counter + 1
	if counter == numberOfUAVs:
		findNeighbors()

def callback0(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback1(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback2(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback3(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback4(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback5(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback6(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback7(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback8(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback9(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback10(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback11(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback12(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback13(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback14(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def callback15(data):
	global flock_data
	#rospy.loginfo("%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)

	if data.ID+":" not in str(flock_data): 
		flock_data = str(flock_data)+str(data.ID)+":"+str(data.x)+","+str(data.y)+","+str(data.z)+","+str(data.vx)+","+str(data.vy)+","+str(data.vz)+","+str(data.yaw)+";"
	checkAllDataReceived()

def listener():
	global subscriber_msg_flag

	rospy.init_node('subscriber_node', anonymous=True)
	#need to subscribe all UAVs

	rospy.Subscriber('talking_topic_ardrone0', FlockMsg, callback0)
	rospy.Subscriber('talking_topic_ardrone1', FlockMsg, callback1)
	rospy.Subscriber('talking_topic_ardrone2', FlockMsg, callback2)
	rospy.Subscriber('talking_topic_ardrone3', FlockMsg, callback3)
	rospy.Subscriber('talking_topic_ardrone4', FlockMsg, callback4)
	rospy.Subscriber('talking_topic_ardrone5', FlockMsg, callback5)
	rospy.Subscriber('talking_topic_ardrone6', FlockMsg, callback6)
	rospy.Subscriber('talking_topic_ardrone7', FlockMsg, callback7)
	rospy.Subscriber('talking_topic_ardrone8', FlockMsg, callback8)
	rospy.Subscriber('talking_topic_ardrone9', FlockMsg, callback9)
	rospy.Subscriber('talking_topic_ardrone10', FlockMsg, callback10)
	rospy.Subscriber('talking_topic_ardrone11', FlockMsg, callback11)
	rospy.Subscriber('talking_topic_ardrone12', FlockMsg, callback12)
	rospy.Subscriber('talking_topic_ardrone13', FlockMsg, callback13)
	rospy.Subscriber('talking_topic_ardrone14', FlockMsg, callback14)
	rospy.Subscriber('talking_topic_ardrone15', FlockMsg, callback15)

	if subscriber_msg_flag == 0:
		rospy.loginfo('Subscriber Started')
		subscriber_msg_flag = 1

	rate = rospy.Rate(1)
	rate.sleep()

	#time.sleep(5)
	#cmd = "rostopic pub /sensor_node std_msgs/String uav0:1,1,1,1,1,1,111;uav1:1,1,1,1,1,1,111;uav2:1,1,1,1,1,1,111;uav3:1,1,1,1,1,1,111"
	#os.system(cmd)

	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
