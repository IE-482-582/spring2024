#!/usr/bin/env python

#Written By: Jack Wuerfel
#Github name: JackWuerfelUB
#Date:3/12/24

#Homework Problem Description:

#how to run..
'''
Assumptions, when running:

To run the code the user needs to have 3 terminals open. Past the following commands in each terminal.

Terminal 1:
roslaunch husky_gazebo husky_playpen.launch

Terminal 2:
change directory to location the script below is saved to.
~/Desktop$ python3 Homework1_Jack_Wuerfel-testnewsavearea.py


Terminal 3:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0 _key_timeout:=0.6 cmd_vel:=dumdum_cmd_vel _speed:=0.9 _turn:=0.8

Helpful commands for troubleshooting:
rostopic echo /cmd_vel
'''




'''
Teleoperate a Husky without running into obstacles.

Write one Python ROS node (.py script) that will do the following:

    Listen to keyboard-input teleop commands (i.e., Twist commands) published by the teleop_twist_keyboard node.
        It is your responsibility to specify the input parameters when issuing the rosrun teleop_twist_keyboard teleop_twist_keyboard.py command.

    Subscribe to the Husky's "scan" topic. Use this info to determine if you will accept the user's Twist command.

    Optionally, you may subscribe to the Husky's "odometry" topic.

    You may use the IE_tools package.

    You will need to specify clearance tolerances; the parameter server is great for that purpose).

    DO NOT EDIT teleop_twist_keyboard.py.

Assumptions:

    The Husky is not moving backwards. In other words, linear.x >= 0. angular.z is unconstrained. You only need to subscribe to the front scanner.
    
    You do not need to edit the incoming Twist command; simply ignore unacceptable commands, publish acceptable ones.

'''
#Notes about this node
'''
This is a very crude node that assists the user while they teleoperate a husky. For this to work
the user must pass in a low linear and turn speed when running the teleop package. It does not do
anything fancy, but it does meet the requirements of not running into objects.
'''

import rospy
import numpy as np

#from nav_msgs.msg import Odometry #prob dont need this one
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist

import sys, os
sys.path.append(f"{os.environ['HOME']}/Projects/IE-482-582/spring2024/Projects/IE_tools")
import IE_tools as IE_tools
import time
from sensor_msgs.msg import LaserScan

#--------------------   input params ----------------------------------------#
CMD_VEL_RATE = 1 #make this a paramater thingy, actually this might not need to be here?? idk it's staying

OBSTACLE_TOLERANCE = rospy.get_param("~obstacle_tolerance",0.75) #[unit is m], default obstacle tolerance is 0.75m
#--------------------   input params ----------------------------------------#





class Keyboard_helper():
    def __init__(self):

        #initialize node
        rospy.init_node("keyboard_helper", anonymous=True)
        
        #this is where the subscription to the dummy cmd happens
        #the "dumdum_cmd_vel" is a parameter passed in when teleop keyboard package is launched
        rospy.Subscriber("/dumdum_cmd_vel", Twist, self.callback_dumdumvel)  #dummy

        
        # Initialize a converter from LaserScan to XY
        self.scan2xy = IE_tools.Scan2XY(scanTopicName   = "/front/scan", 
										refFrame        = 'FLU', 
										userAngleMinDeg = -85, 		# Far RIGHT
										userAngleMaxDeg = +85)		# Far LEFT
        
        #subscribe to laser here (alot of this is taken from code "laser_scan_subscriber.py"), Credit: Dr. Murray
        rospy.Subscriber("/front/scan", LaserScan, self.callback_front_scan)	

        #publisher for husky's cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        #testing
        self.rate = rospy.Rate(CMD_VEL_RATE)	# [Hz]   ()
        
        #intitialize "flag" variable as true
        self.Safe_Var = True

        #keeps program running
        self.run()
        
    def run(self):
         '''
         This function keeps the program running.
         '''
         rospy.spin()

    def callback_dumdumvel(self, msg):
        #testing - trying to get info from dumdum cmd vel and translate it to normal cmd vel
        #x = 1 #get x component of dumdum_cmd_vel
        #z = 1 #get z component of dumdum_cmd_vel
        
        #print("is this even working")
        #if  np.min(self.x_array) < OBSTACLE_TOLERANCE:  #if statement didnt really work

        if(self.Safe_Var == True):  
            twistMsg = self.createTwistMsg(msg.linear.x, msg.angular.z)						
        
            #call eval function here???

            self.cmd_vel_pub.publish(twistMsg)
       
    def callback_front_scan(self, msg):
		# print(msg.angle_min, msg.ranges[0])

		# Convert LaserScan data to (x,y)
        self.scan2xy.scan2xy(msg)

        '''
		# If an obstacle is detected, just print the first result:
		if (len(self.scan2xy.x) > 0):
			print(self.scan2xy.x[0], self.scan2xy.y[0])
		'''

		# QUESTION:  What does this do (assuming `refFrame == 'FLU'`)?	
		# huskyWidth = 0.67	meters
        self.x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
        if (len(self.x_array) > 0):
            print(np.min(self.x_array))
            if np.min(self.x_array) > obstacle_tolerance:
                 print("robot is safe to move")
                 self.Safe_Var = True
            elif np.min(self.x_array) < obstacle_tolerance:
                print("robot unsafe to move")
                self.Safe_Var = False
        


    def eval_cmd(self, msg):
        #testing


        #testing
        return()

    def createTwistMsg(self, linearX=0, angularZ=0):   #prob dont need this function???
        '''
        This function returns a Twist message.
        You'll need to decide: 
        1) What info to pass to this function, and
        2) What the values for linear.x and angular.z should be.
        '''
        msg = Twist()

        msg.linear.x  = linearX
        msg.angular.z = angularZ

        return msg

if __name__ == "__main__":
	Keyboard_helper()