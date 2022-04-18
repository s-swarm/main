#python generate_flockLaunch [number_of_UAVs] [minimum_distance_between_each_other]

import sys
import math

N = int(sys.argv[1])
minDist = int(sys.argv[2])

f_name = "flock.launch"
f = open(f_name, "w")

f.write("<launch>\n")

f.write("\n")
f.write("  <param name=\"/use_sim_time\" value=\"true\"/>\n")
f.write("\n")

f.write("  <arg name=\"mav_name\" default=\"firefly\"/>\n")
f.write("  <arg name=\"world_name\" default=\"basic\"/>\n")
f.write("  <arg name=\"enable_logging\" default=\"false\"/>\n")
f.write("  <arg name=\"enable_ground_truth\" default=\"true\"/>\n")
f.write("  <arg name=\"log_file\" default=\"$(arg mav_name)\"/>\n")
f.write("  <arg name=\"debug\" default=\"false\"/>\n")
f.write("  <arg name=\"gui\" default=\"true\"/>\n")
f.write("  <arg name=\"paused\" default=\"true\"/>\n")
f.write("  <arg name=\"verbose\" default=\"false\"/>\n")

f.write("\n")
f.write("  <env name=\"GAZEBO_MODEL_PATH\" value=\"${GAZEBO_MODEL_PATH}:$(find rotors_gazebo)/models\"/>\n")
f.write("  <env name=\"GAZEBO_RESOURCE_PATH\" value=\"${GAZEBO_RESOURCE_PATH}:$(find rotors_gazebo)/models\"/>\n")
f.write("  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">\n")
f.write("    <arg name=\"world_name\" value=\"$(find rotors_gazebo)/worlds/$(arg world_name).world\"/>\n")
f.write("    <arg name=\"debug\" value=\"$(arg debug)\" />\n")
f.write("    <arg name=\"paused\" value=\"$(arg paused)\" />\n")
f.write("    <arg name=\"gui\" value=\"$(arg gui)\" />\n")
f.write("    <arg name=\"verbose\" value=\"$(arg verbose)\"/>\n")
f.write("  </include>\n")

uavLocations = [[0 for i in range(3)] for j in range(N)]

temp = int(((math.sqrt(N) - 1) * minDist))
uavCounter = 0
for j in range (temp, -1, -minDist):
	for k in range (0, temp+1, minDist):
		uavLocations[uavCounter][0] = j - (int(math.sqrt(N) / 2) * minDist) #posX
		uavLocations[uavCounter][1] = k - (int(math.sqrt(N) / 2) * minDist) #posY
		uavCounter = uavCounter + 1

for i in range (0,N):
	f.write("\n")
	f.write("  <group ns=\"$(arg mav_name)"+str(i)+"\">\n")
	f.write("    <include file=\"$(find rotors_gazebo)/launch/spawn_mav.launch\">\n")
	f.write("      <arg name=\"mav_name\" value=\"$(arg mav_name)"+str(i)+"\" />\n")
	f.write("      <arg name=\"model\" value=\"$(find rotors_description)/urdf/$(arg mav_name)_base.xacro\" />\n")
	f.write("      <arg name=\"enable_logging\" value=\"$(arg enable_logging)\" />\n")
	f.write("      <arg name=\"enable_ground_truth\" value=\"$(arg enable_ground_truth)\" />\n")
	f.write("      <arg name=\"log_file\" value=\"$(arg log_file)\"/>\n")
		
	f.write("      <arg name=\"x\" value=\""+str(uavLocations[i][0])+"\"/>\n")
	f.write("      <arg name=\"y\" value=\""+str(uavLocations[i][1])+"\"/>\n")
	f.write("    </include>\n")
	f.write("  </group>\n")    

f.write("\n")
f.write("</launch>")    

f.close()
