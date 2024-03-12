#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan


from geometry_msgs.msg import Twist
import time


import sys
sys.path.append("..")
import IE_tools.IE_tools as IE_tools

import numpy as np

class HuskyProtector():
	def __init__(self):
	
		rospy.init_node("laser_scan_reader",)

		# NEW
		# Initialize a converter from LaserScan to XY
		self.scan2xy = IE_tools.Scan2XY(scanTopicName   = "/front/scan", 
										refFrame        = 'FLU', 
										userAngleMinDeg = -85, 		# Far RIGHT
										userAngleMaxDeg = +85)		# Far LEFT

		# rospy.Subscriber("<topic name>", <topic type>, <callback>)	
		rospy.Subscriber("/front/scan", LaserScan, self.callback_front_scan)	
		rospy.Subscriber("/cmd_vel", TwistMsg, self.callback_teleop)

		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.rate = rospy.Rate(1/5)	# [Hz]
		self.raw_twist.linear.x = 0
		self.raw_twist.angular.z = 0

	
		rospy.spin()
		
	def callback_front_scan(self, msg):
		# print(msg.angle_min, msg.ranges[0])

		# Convert LaserScan data to (x,y)
		self.scan2xy.scan2xy(msg)


		x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
		if (len(x_array) > 0):
			print(np.min(x_array))
		
	def callback_teleop(self, msg):
	
		self.raw_twist.linear.xself.raw_twist.linear.x = self.x * self.speed
		self.raw_twist.angular.z = self.th * self.turn
	
	def run(self):
		
		if np.min(x_array) > speed:
			linearX  = self.raw_twist.linear.x
			angularZ = self.raw_twist.angular.z
		else:
			linearX = 0
			angularZ = 0
		twistMsg = self.createTwistMsg(linearX, angularZ)						
		self.cmd_vel_pub.publish(twistMsg)

		self.rate.sleep()



if __name__ == "__main__":
	HuskyProtector()