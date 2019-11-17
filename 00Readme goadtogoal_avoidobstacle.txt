
To run the python code, a catkin package was created. To do that, the current directory was set to the source space directory of the catkin workspace.

$ cd ~/catkin_ws/src


The package was created by Using the catkin_create_pkg script.
It requires a package_name and optionally a list of dependencies on which the package depends. 
I set the name of the package as laser_class.The node is a python script, so "rospy".   
   
$ catkin_create_pkg laser_class rospy 

Following commands were used to build the package in the catkin workspace. 

$ cd ~/catkin_ws
$ catkin make

To add the workspace to ROS environment, setup file was sourced as following:

$ source ~/catkin_ws/devel/setup.bash


The python file was placed inside a script folder of the created package. 

To make this file executable for ROS, run the following command from the script folder:

$ chmod +x gotofoal_avoidobstacle.py


To run the the node, execute the following 3 commands in 3 different terminal
$ roscore

$ roslaunch husky_gazebo husky_empty_world.launch

$ rosrun laser_class gotogoal_avoidobstacle.py

It will ask the user to set destination coordinate. After setting the coordinate, the husky will move to that location 
avoiding obstacles in its path and stop when it will reach within 10cm of the destination. 


 
