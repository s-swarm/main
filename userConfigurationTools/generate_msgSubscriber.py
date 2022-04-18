#python generate_msgSubscriber.py [number_of_UAVs]

import sys
import math

N = int(sys.argv[1])

f = open("msgSubscriber.py", "w")

f.write("#python uavMsgSubscriber.py [uav_number]\n")
f.write("#!/usr/bin/env\n")
f.write("\n")

f.write("import rospy, sys\n")
f.write("from rotors_gazebo.msg import FlockMsg\n")
f.write("from std_msgs.msg  import String\n")
f.write("import time\n")
f.write("import os\n")
f.write("\n")

f.write("#published message format\n")
f.write("#rostopic pub /sensor_node std_msgs/String \"uav0:1,1,1,1,1,1,111;uav1:1,1,1,1,1,1,111;uav2:1,1,1,1,1,1,111;uav3:1,1,1,1,1,1,111\"\n")
f.write("\n")

f.write("numberOfUAVs = int(sys.argv[1])\n")
f.write("subscriber_msg_flag = 0\n")
f.write("pub = rospy.Publisher('sensor_node', String, queue_size=10)\n")
f.write("flock_data = \"\"\n")
f.write("\n")

f.write("def findNeighbors():\n")
f.write("\tglobal flock_data\n")
f.write("\n")
f.write("\tpub.publish(flock_data)\n")
f.write("\tprint(\"..flock_data published..\")\n")
f.write("\tprint(flock_data)\n")
f.write("\tflock_data = \"\"\n")
f.write("\n")

f.write("def checkAllDataReceived():\n")
f.write("#check flock_data; if got data from all UAVs, then call findNeighbors function.\n")
f.write("\tcounter = 0\n")
f.write("\tfor i in range(0, numberOfUAVs):\n")
f.write("\t\tif \"ardrone\"+str(i)+\":\" in str(flock_data):\n")
f.write("\t\t\tcounter = counter + 1\n")
f.write("\tif counter == numberOfUAVs:\n")
f.write("\t\tfindNeighbors()\n")
f.write("\n")

for i in range (0, N):
	f.write("def callback"+str(i)+"(data):\n")
	f.write("\tglobal flock_data\n")
	f.write("\t#rospy.loginfo(\"%s => X: %f Y: %f Z: %f vx: %f vy: %f vz: %f yaw: %f\", data.ID, data.x, data.y, data.z, data.vx, data.vy, data.vz, data.yaw)\n")
	f.write("\n")
	f.write("\tif data.ID+\":\" not in str(flock_data): \n")
	f.write("\t\tflock_data = str(flock_data)+str(data.ID)+\":\"+str(data.x)+\",\"+str(data.y)+\",\"+str(data.z)+\",\"+str(data.vx)+\",\"+str(data.vy)+\",\"+str(data.vz)+\",\"+str(data.yaw)+\";\"\n")
	f.write("\tcheckAllDataReceived()\n")
	f.write("\n")

f.write("def listener():\n")
f.write("\tglobal subscriber_msg_flag\n")
f.write("\n")
f.write("\trospy.init_node('subscriber_node', anonymous=True)\n")
f.write("\t#need to subscribe all UAVs\n")
f.write("\n")

for i in range (0, N):
	f.write("\trospy.Subscriber('talking_topic_ardrone"+str(i)+"', FlockMsg, callback"+str(i)+")\n")

f.write("\n")
f.write("\tif subscriber_msg_flag == 0:\n")
f.write("\t\trospy.loginfo('Subscriber Started')\n")
f.write("\t\tsubscriber_msg_flag = 1\n")
f.write("\n")

f.write("\trate = rospy.Rate(1)\n")
f.write("\trate.sleep()\n")
f.write("\n")

f.write("\t#time.sleep(5)\n")
f.write("\t#cmd = \"rostopic pub /sensor_node std_msgs/String uav0:1,1,1,1,1,1,111;uav1:1,1,1,1,1,1,111;uav2:1,1,1,1,1,1,111;uav3:1,1,1,1,1,1,111\"\n")
f.write("\t#os.system(cmd)\n")
f.write("\n")

f.write("\trospy.spin()\n")
f.write("\n")

f.write("if __name__ == '__main__':\n")
f.write("\ttry:\n")
f.write("\t\tlistener()\n")
f.write("\texcept rospy.ROSInterruptException:\n")
f.write("\t\tpass\n")
f.write("\n")

f.close()
