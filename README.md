# Drone_simulator
Drone simulator visiting locations based on Aruco markers.
This repository uses Robot Operating System and Gazebo simulator.

## Overview
The drone has a goal to visit all locations for which it knows the general coordinates. The control is estimated by calculating the course between the target location and the drone's GPS coordinates. Each of the drone landing fields to which the drone is directed has an Aruco marker. 
When a flying object sees a marker, it moves from the flight phase to the landing phase, being positioned relative to the location of the marker in the down camera image. 

The main packages that are used include:
- **tum_simulator** -  is responsible for generating the Gazebo environment and physics of the objects during the simulation,
- **ardrone_joystick** -  allows to control the drone manually with the joystick for Xbox One,
- **nswr_gps** - determines the GPS coordinates of the drone and converts them into UTM coordinates,
- **drone_control** - is responsible for the autonomous flight of the drone. The drone_controller.py file contains an interface to send flight commands and set angular velocities.


# Installation of ROS environment
A Docker image was used to facilitate the installation of the ROS Kinetic environment and the TUM Simulator package.

In order to download it, you need to have the Docker environment installed and run the command:

```
docker pull kroszyk/tum-simulator-gpu-support:latest
```
If you want to switch to a container in a new terminal use the command:
```
docker exec -it drone_simulator bash
```

After downloading the docker image, clone the Github repository to the selected folder, using the command:
```
git clone https://github.com/KRoszyk/Drone_simulator
```
Once the repository files are downloaded. navigate to the catkin_ws folder and build the ROS environment:
```
cd catkin_ws 
catkin_make
```
If the environment build was successful, you can start running the environment. Remember to use the command in the catkin_ws folder when opening each new terminal in the container:
```
source devel/setup.bash
```
# Commands to run the environment
To launch the Gazebo simulator from the cvg_sim_gazebo package, use the command:
```
roslaunch cvg_sim_gazebo ardrone_testworld.launch
```
The Gazebo environment should turn on with a drone created in the middle.

Next, activate the package responsible for converting the drone's GPS coordinates into local ones.
```
roslaunch nswr_gps nswr_gps.launch 
```
As a final step, you can run the package responsible for flying the drone and landing it on markers relative to a specific order.
```
roslaunch drone_control drone.launch 
```
It is also possible to control the drone with the Xbox One joystick by using a command:
```
roslaunch ardrone_joystick teleop.launch
```
Note, however, that joystick control is not compatible with autonomous drone flight and you should always disable one of the nodes responsible for manual or automatic control.

# Simulator performance results
The following screenshot shows the bottom camera image during the landing phase and the detection of the Aruco marker. 

![Screenshot](https://github.com/KRoszyk/Drone_simulator/blob/master/imgs/marker_detected.png)

In addition, a video of a drone flying between locations defined by Aruco markers is presented below.
[![Alt text](https://img.youtube.com/vi/1fQAQxSL0_g/0.jpg)](https://youtu.be/1fQAQxSL0_g)

# Contributors
[Kamil Roszyk](https://github.com/KRoszyk)

[Witold Sempruch](https://github.com/Witsemp)

[Jarosław Bożek](https://github.com/JaroslawBozek)
