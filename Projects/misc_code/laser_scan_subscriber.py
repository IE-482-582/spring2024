#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
# The message **type** is sensor_msgs/LaserScan
# See output from `rostopic info /front/scan`

# from geometry_msgs.msg import Twist
import time

# Our support script `IE_tools.py` is in a parallel directory.
# So, we have to let Python know where to find it:
import sys
sys.path.append("..")
import IE_tools.IE_tools as IE_tools

import numpy as np

class LaserScanReader():
	def __init__(self):
	
		rospy.init_node("laser_scan_reader")

		# NEW
		# Initialize a converter from LaserScan to XY
		self.scan2xy = IE_tools.Scan2XY(scanTopicName   = "/front/scan", 
										refFrame        = 'FLU', 
										userAngleMinDeg = -85, 		# Far RIGHT
										userAngleMaxDeg = +85)		# Far LEFT

		# rospy.Subscriber("<topic name>", <topic type>, <callback>)	
		rospy.Subscriber("/front/scan", LaserScan, self.callback_front_scan)	
		
		# self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		# self.rate = rospy.Rate(1/5)	# [Hz]
		# self.run()
	
		rospy.spin()
		
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
		x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
		if (len(x_array) > 0):
			print(np.min(x_array))
		
	'''	
	def run(self):
		# THIS IS BAD FORM!
		while True:
			self.rate.sleep()
	'''		
			
if __name__ == "__main__":
	LaserScanReader()
