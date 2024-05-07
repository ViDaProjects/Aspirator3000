# Aspirator3000

Aspirator 3000: Robotic Vacuum Cleaner

## Steps to clone the repository

### For ROS beginners

  <p> 1- Install ROS by this tutorial: https://www.youtube.com/watch?v=TnE7-tJOJjo&t=11s </p>
  
  <p> 2- Create and setup Catkin Workspace (https://www.youtube.com/watch?v=8uxd9RBQvmQ): </br>
      ~$ mkdir catkin_ws </br>
      ~$ cd catkin_ws/ </br>
      ~/catkin_ws$ mkdir src </br>
      ~/catkin_ws$ catkin_make </br>
      ~/catkin_ws$ cd .. </br>
      ~$ source ~/catkin_ws/devel/setup.bash </br>
      ~$ gedit ~/.bashrc  </br>
       Obs: Insert at the end of the file .bashrc: source ~/catkin_ws/devel/setup.bash </p>
  
  <p> 3- Cloning the repository: </br>
      ~$ cd catkin_ws/src/ </br>
      ~/catkin_ws/src$ git clone https://github.com/ViDaProjects/aspirator3000.git </br>
      ~/catkin_ws/src$ cd .. </br>
      ~/catkin_ws$ catkin_make </p>
      
      
      
