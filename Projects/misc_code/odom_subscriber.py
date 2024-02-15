#!/usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

TWO_PI = 2*np.math.pi

class OdomReader():
	def __init__(self):
	
		rospy.init_node("odometry_reader")
	
		rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)
		
		
		rospy.spin()
		'''
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			r.sleep()
		'''

	def callback_odometry(self, msg):

		p = msg.pose.pose

		orientation_list = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

		self.yaw = self.yaw % TWO_PI
		self.heading = round(np.rad2deg(self.yaw), 1) 
		
		self.pos_body_x_m  = round( p.position.x, 3)		# True?
		self.pos_body_y_m  = round( p.position.y, 3)		# True?
		self.pos_body_z_m  = round( p.position.z, 3)		# True?
		
		self.vel_body_x_m_s = round(msg.twist.twist.linear.x, 2)    # True?
		# Note:  linear.y is always 0.  Also, angular.x/.y always 0. 
		# The values published here might be aspirational (as opposed to actual).
				
		# print(self.roll, self.pitch, self.yaw)
		print(self.pos_body_x_m, self.pos_body_y_m, self.pos_body_z_m, self.heading)
			
if __name__ == "__main__":
	OdomReader()
