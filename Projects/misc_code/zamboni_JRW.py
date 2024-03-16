#!/usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist

# Our support script `IE_tools.py` is in a parallel directory.
# So, we have to let Python know where to find it:
import sys
sys.path.append("/home/jack/Projects/IE-482-582/spring2024/Projects/IE_tools")
import IE_tools.IE_tools as IE_tools

#example call from ie tools self.scan2xy = IE_tools.Scan2XY()




# ----------------------------------------
CMD_VEL_RATE = 10  # [Hz]

TWO_PI = 2*np.math.pi

USER_INPUT_L =  20 #longside
USER_INPUT_S = 5 #shortside
USER_INPUT_W = 100 #total width of room
USER_MAX_LINEAR_X = 2 #max speed husky can ever go
USER_MAX_ANGULAR_Z = 0.25 #max 'speed' husky can turn
# ----------------------------------------

class Zamboni():
	def __init__(self):
	
		rospy.init_node("zamboni_driver", anonymous=True)
	
		rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)	

		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.rate = rospy.Rate(CMD_VEL_RATE)	# [Hz]

		# Set the shutdown function
		rospy.on_shutdown(self.shutdown)

		self.run()
		
	def callback_odometry(self, msg):
		'''
		This "callback" function processes the data we 
		receive from the "odometry/filtered" topic.
		'''
		p = msg.pose.pose

		orientation_list = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
		self.yaw = self.yaw % TWO_PI
		self.heading = round(np.rad2deg(self.yaw), 1) 
		
		self.pos_body_x_m  = round( p.position.x, 3)	
		self.pos_body_y_m  = round( p.position.y, 3)	
		# self.pos_body_z_m  = round( p.position.z, 3)	

		'''
		self.vel_body_x_m_s = round(msg.twist.twist.linear.x, 2)    # True?
		# Note:  linear.y is always 0.  Also, angular.x/.y always 0. 
		# The values published here might be aspirational (as opposed to actual).
				
		# print(self.roll, self.pitch, self.yaw)
		print(self.pos_body_x_m, self.pos_body_y_m, self.pos_body_z_m, self.heading)
		'''
		
	def createTwistMsg(self, linearX=0, angularZ=0):
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
			
	def run(self):
		'''
		This is the function that will publish Twist commands
		'''
		
		while not rospy.is_shutdown():

			# Do some "stuff" here...

			# Call a function that will craft a
			# Twist message to control the robot.
			# FIXME -- Need to decide what info 
			# gets passed to the function.
			twistMsg = self.createTwistMsg()
			
			self.cmd_vel_pub.publish(twistMsg)

			self.rate.sleep()
				
	def shutdown(self):
		'''
		This function will be called when the script is ended.
		This gives us a chance to shut things down cleanly.
		'''
		
		print("Hey, I'm shutting down now")
		
		twistMsg = Twist()	
		self.cmd_vel_pub.publish(twistMsg)

		print("I told the robot to stop moving.  Bye")
		
    def generate_path_xy_coords(self):
	    '''
		USER_INPUT_L =  20 #longside
        USER_INPUT_S = 5 #shortside
        USER_INPUT_W = 100 #total width of room
		
		Generate a list of (x, y) coordinates for a robot's path given the forward 
		movement distance L, the total length S, and the total width W of the room.
    
        Parameters:
        L (int): Distance the robot moves forward in each step.
        S (int): Total length of the room.
        W (int): Total width of the room.
    
        Returns:
        list of tuples: A list of (x, y) coordinate pairs representing the robot's path.
		'''
        x, y = 0,0
        
        
        
        
        
        
        self.test = 3
		
if __name__ == "__main__":
	Zamboni()
