#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
# The message **type** is sensor_msgs/LaserScan
# See output from `rostopic info /front/scan`

# from geometry_msgs.msg import Twist
import time

# Our support script `IE_tools.py` is in a parallel directory.
# So, we have to let Python know where to find it:
import sys,os
sys.path.append("..")
import IE_tools.IE_tools as IE_tools

import numpy as np

class OAfilter():
	def __init__(self):
	
		rospy.init_node("laser_scan_reader", anonymous = True)

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

		self.scan2xy.scan2xy(msg)
		x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
		if (len(x_array) > 0):
			print(np.min(x_array))
		
	
			
if __name__ == "__main__":
	OAfilter()
