# Aspirator3000

Aspirator 3000 is a cleaning robot much like the ones you find at home, this one, however, comes with a bit of a twist, it can guarantee a quick and simple cleaning session through the use of a complex navigation system. With a LIDAR as well as encoder motors it maps the environment it will clean and proceeds to sweep the area in an efficient manner.

## Installation

```sh

sudo apt install ros-noetic-gmapping

cd ~/catkin_ws/src
git clone https://github.com/ViDaProjects/aspirator3000.git
git clone https://github.com/Slamtec/rplidar_ros.git

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Running

### Simulation

Launching the simulation (currently on an empty map).

```sh
roslaunch gazebo_aspirator aspirator_sim.launch
```

![Gazebo model](https://github.com/ViDaProjects/aspirator3000/blob/main/images/simulation.png?raw=true)

### Real robot

All this will be changed to a single roslaunch when I get the chance

```sh
roslaunch rplidar_ros rplidar_a1.launch
```
