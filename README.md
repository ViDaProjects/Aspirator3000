# Aspirator3000
Aspirator 3000: Robotic Vacuum Cleaner

# Steps to clone the repository:
  1- Install ROS by this tutorial: https://www.youtube.com/watch?v=TnE7-tJOJjo&t=11s
  2- Create and setup Catkin Workspace ():
      ~$ mkdir catkin_ws
      ~$ cd catkin_ws/ 
      ~/catkin_ws$ mkdir src
      ~/catkin_ws$ catkin_make
      ~/catkin_ws$ cd ..
      ~$ source ~/catkin_ws/devel/setup.bash
      ~$ gedit ~/.bashrc  
       Obs: Insert at the end of the file .bashrc: source ~/catkin_ws/devel/setup.bash
  3- Cloning the repository:
      ~$ cd catkin_ws/src/
      ~/catkin_ws/src$ git clone https://github.com/ViDaProjects/aspirator3000.git
      ~/catkin_ws/src$ cd ..
      ~/catkin_ws$ catkin_make
      
      
      
