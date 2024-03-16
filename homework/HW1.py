#!/usr/bin/env python3

import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#CUSTOM PARAMETERS...MAKE TURN = 0.5

class HW1():
	def __init__(self):
		
		#Initialize
		rospy.init_node("HW1", anonymous = True)
		self.closest_object = 0
		self.command = Twist()
		self.guardian = 0 #flag for whether or not to move
		
		#Subscribe	
		rospy.Subscriber("/front/scan", LaserScan, self.callback_front_scan)	
		rospy.Subscriber("/cmd_vel", Twist, self.callback_teleop)	
		
		#Publish
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.run()
		
	def callback_teleop(self, msg):
		self.command.angular.z = msg.angular.z
		if (self.guardian == 1):
			self.command.linear.x = msg.linear.x
	
	def callback_front_scan(self, msg):
		min_angle = msg.angle_min
		max_angle = msg.angle_max
		increment = msg.angle_increment
		ranges = list(msg.ranges)

		
		cone = max_angle - min_angle
		total_scans = math.ceil(cone/increment)
		target_start = math.ceil(0.25*total_scans)
		target_end = math.ceil(total_scans - target_start)
		
		target_range = ranges[target_start:target_end]
		self.closest_object = min(target_range)
		
		if (self.closest_object <= 3): #clearance
			self.guardian = 0
			self.command.linear.x = 0
			self.cmd_vel_pub.publish(self.command)
			print("Turn")
		else: 
			self.guardian = 1
			self.cmd_vel_pub.publish(self.command)
			print("You can move")
		
		
	def run(self):
		while not rospy.is_shutdown():
			self.cmd_vel_pub.publish(self.command)
		

		
if __name__ == "__main__":
	HW1()
		
		
