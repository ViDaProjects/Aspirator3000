# Aspirator3000

Aspirator 3000: Robotic Vacuum Cleaner

## Steps to clone the repository

### For ROS beginners

1. Use ubuntu 20.04. NOTHING ELSE

2. [Install ROS by this tutorial](https://www.youtube.com/watch?v=TnE7-tJOJjo&t=11s) or run `wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh`
  
3. [Create and setup Catkin Workspace](https://www.youtube.com/watch?v=8uxd9RBQvmQ): </br>
      ~$ mkdir catkin_ws </br>
      ~$ cd catkin_ws/ </br>
      ~/catkin_ws$ mkdir src </br>
      ~/catkin_ws$ catkin_make </br>
      ~/catkin_ws$ cd .. </br>
      ~$ source ~/catkin_ws/devel/setup.bash </br>
      ~$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

4. Cloning the repository: </br>
      ~$ cd catkin_ws/src/ </br>
      ~/catkin_ws/src$ git clone https://github.com/ViDaProjects/aspirator3000.git </br>
      ~/catkin_ws/src$ cd .. </br>
      ~/catkin_ws$ catkin_make