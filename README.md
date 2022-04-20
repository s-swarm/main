S-Swarm is an open source simulation platform for quadrotor swarms. It is developed utilizing some powerful open-source robotics tools such as ROS, Gazebo and RotorS simulator.

It provides low-level and high-level flight controllers based on PID control approach, and a communication module for exchanging information among the agents in swarm. In addition, several example swarm behaviors  are also developed and put into swarm behaviors module.

This platform also contains some user configuration tools such as launch file generator, sensing & communication system builder, concurrent flight controller runner and flight data logger files. 

Below we provide the instructions necessary for getting started. 

## Requirements:

#### Install ROS:
We recommend for these ROS integration tutorials you install (ros-noetic-desktop-full) so that you have all the necessary packages. See the [ROS installation page](http://wiki.ros.org/noetic/Installation/Ubuntu) for more details. Be sure to source your ROS setup.bash script by following the instructions on the ROS installation page.

#### Install Gazebo:

See [Install Gazebo page](http://gazebosim.org/tutorials). If installing from source, be sure to build the gazebo_X.Y (X.Y being your desired version) branch.

#### Install RotorS:

[Get the simulator](https://github.com/ethz-asl/rotors_simulator) and additional dependencies and build your workspace.

## Getting the S-Swarm ready to use:
1. [Get](https://bit.ly/38TBihQ) the world file to your workspace location *~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds/basic.world*

2. Set the launch file.

	- [Get](https://bit.ly/3uZ8YDy) the launch file generator. Please run:
	   
      `$python generate_flockLaunch.py N d`
     
      > N: number of drones, d: distance (m) between each robot in starting position of the experiment.

	- Copy the generated 'flock.launch' file to your workspace location *~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch/flock.launch*

3. Set the sensing and communication system.

	- Create a *'msg'* directory to your workspace location *~/catkin_ws/src/rotors_simulator/rotors_gazebo/msg* and copy *'FlockMsg.msg'* file [here](https://bit.ly/3L2AJR0). 

	- [Get](https://bit.ly/3xEFaxC) the sensing & communication system builder file. Please run:
	 
	  `$python generate_msgSubscriber.py N #N: number of drones.`

## Using S-Swarm:
1. roscore is the first thing you should run when using ROS. Please run:

	`$roscore`

2. Launch the simulator with a quadrotor model, in our case, the Ardrone in a basic world. Please run:

	`$roslaunch rotors_gazebo mav.launch mav_name:=ardrone world_name:=basic`

3. Go to downloaded 's-swarm/swarmController' directory and please run:

	`$python msgSubscriber.py N`
    > N: number of drones.

4. Go to downloaded 's-swarm/userConfigurationTools' directory and please run:

	`$python runner.py N r `
    > N: number of drones, r: metric distance sensing radius (m) for each robot.`
