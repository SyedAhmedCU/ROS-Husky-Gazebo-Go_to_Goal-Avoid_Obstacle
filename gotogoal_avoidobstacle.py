#!/usr/bin/env python

#writing this on the first line 
#ensures that this file is executed as a python script

import rospy		#Importing rospy package Ros python node script
import numpy as np
from math import sqrt, atan2
from geometry_msgs.msg import Twist	#for cmd_vel topic
from nav_msgs.msg import Odometry	#for /odometry/filtered topic
from sensor_msgs.msg import LaserScan	#for /scan topic to use LIDAR
from tf.transformations import euler_from_quaternion

x=0
y=0
theta=0 #yaw
ranges=0 
min_dist=0 
angle=0 

def update_odom(odom_data):
    global x,y,theta  
    #set position and orientation values from odometry to global variables     
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    rot = odom_data.pose.pose.orientation 
 
    #calculating orientation theta (yaw) from quaternion   
    (roll, pitch, theta) = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])     

def laserscan_update(laser_data):
    global min_dist, angle 
    
    #distances from laser 
    ranges = laser_data.ranges

    # convert to numpy array to be able to use numpy functions
    npranges = np.array(ranges)

    # compute minimum distance and its corresponding angles with respect to scanner's frame
    min_dist = np.nanmin(npranges)
    indices = np.reshape( np.argwhere(npranges == min_dist) , -1)
    angle_array = ((indices*laser_data.angle_increment)+laser_data.angle_min)*180.0/np.pi
    angle= angle_array.item(0) #value of laser angle
        
def husky_nav(goal_x, goal_y):
    
    # creates an Rate object rate, which when combined with sleep() method on 
    # line 70 offers a convenient way of publishing data at a 
    # desired rate (frequency), or this case, its 10Hz,
    # data is published on cmd_vel topic 10 times per second

    r=rospy.Rate(10)

    # setting up a variable of massage type Twist for velocity components 

    vel = Twist()
        
    while not rospy.is_shutdown():
       
       #A proportional controller was used to correct angular velocity 
       #by multiplying a contant value to the orientation error. 
       #set the value of this controller 
       #tuned it by trial and error 

       kp=1 		

       #finds the distance between the bot and the target location 
       #Euclidean distance method was used

       goal_dist = abs(sqrt((goal_x-x)**2+(goal_y-y)**2))

	#finds the orientation required to turn the bot towards target location
       theta_req = atan2(goal_y-y, goal_x-x) 
  
	# Find error between the bot's heading 
        # and orientation  required for the target location    
       theta_error= (theta_req-theta)   
       
	#distance tolerance between goal and obstacle    
       obs_dist= 1 

	#distance between robot and the obstacle (from laser_scan)	   
       obs = min_dist

	#converting angles from radian to degree for printing on the screen 	
       theta_deg=theta*180/np.pi
       theta_req_deg=theta_req*180/np.pi
       theta_error_deg= theta_error*180/np.pi
	
	#Logic for avoiding obstacle

       if obs <= obs_dist and goal_dist > obs_dist: 
           vel.linear.x = 0.2 #move with a lower velocity around obstacle
           print ('Obstacle nearby, moving slowly!!!')
           
	    # if the angle <0, obstacle's on the left, to 
	    #turn right, angular velocity has to be negative 
	    #and vice versa

           if angle <= 0.0: 
               vel.angular.z= -0.5
               print ('Obstacle on the left, turning right!!!')
           elif angle > 0.0:
               vel.angular.z= 0.5
               print ('Obstacle on the right, turning left!!!')
       
	# if the distance difference is less than 10cm,
        # all velocities become zero to stop the robot       
       elif goal_dist < 0.1:
           vel.linear.x = 0.0   
           vel.angular.z= 0.0
           print ('!!!!! Destination Reached !!!!!')
          
	# logic for go to goal 
	# Controls angular velocity by multiplying the p controller
       elif goal_dist <= obs_dist:
            vel.linear.x = 0.2  #reduce speed to 0.2m/s as it comes closeser to the goal
            vel.angular.z = kp*theta_error               
            print ('Destination nearby!!!')    
            
       elif goal_dist > obs_dist and obs > obs_dist:               
            vel.linear.x = 0.4  #higer velocity when both obstacle and the goal are far
            vel.angular.z = kp*theta_error               
            print ('Destination far away!!!')

       #publishing velocity 
       vel_pub.publish(vel)   
       
       #publishing signals on the screen (as mentioned in the assignment) 
       rospy.loginfo('Target location, x: %s, y:%s', goal_x, goal_y)
       rospy.loginfo('Current location, x: %s, y:%s', x, y)
       rospy.loginfo('Distance error in meter: %s', goal_dist)
       rospy.loginfo('Current Orientation  in degree:%s', theta_deg)
       rospy.loginfo('Target Orientation  in degree: %s', theta_req_deg)
       rospy.loginfo('Orientation error in degree: %s', theta_error_deg)

       # The loop calls rate.sleep(), which sleeps just long enough
       # to maintain the desired rate (20HZ) through the loop.
       r.sleep()          
            
if __name__ == '__main__':
    try:
    	# Initialize the node by giving it name turtle_navigation
   	# anonymous= true , ensures that every node has a different name by 
   	# appending it with a random numbers. 
    	# anonymous= false, as I made sure that this name is unique for this case

        rospy.init_node('husky_nav', anonymous=False)
        
    	# declares that the node is publishing to the /cmd_vel topic
    	# using the message type Twist of the class geometry_msgs.msg.
    	# The queue_size argument limits the amount of queued messages
   	# if any subscriber is not receiving them fast enough

        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size =10 )

	# Subscribe the node to the topic /odometry/filtered and /scan, 
	# which is of type Odometry and LaserScan respectively.
    	# Whenever a new message is published on these topics, 
    	# the functions update_odom and laserscan_update are invoked respectively  
    	# with the messageas its first argument (implicit) 

        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, update_odom)
        laser_sub = rospy.Subscriber('/scan', LaserScan, laserscan_update)

	# Getting input from user for the destination coordinate
        print ('Set coordinates (x,y) of the destination, where 0 <= (x, y) <=10')
        a = input("set x coordinate of your desired location: ") 
        b = input("set y coordinate of your desired location: ")
        
        if 0 <= a <=10 and 0 <= b <=10: #check if the coordinates are in range
            husky_nav(a, b)
        else:
            print ('Warning!! Destination out of range, 0 <= (x, y) <=10')
            print ('Set coordinates again by running the node')
            
    except rospy.ROSInterruptException:
        pass
