#!/usr/bin/env python3
#python3 H1.py

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist

import sys, os
sys.path.append(f"{os.environ['HOME']}/Projects/IE-482-582/spring2024/Projects/IE_tools")
import IE_tools as IE_tools
from sensor_msgs.msg import LaserScan
import time

#----
obstacle_distance_threshold = 1.0  # Set the threshold distance to stop the robot

obstacle_detected = False
'''
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

	# If an obstacle is detected, just print the first result:
	if (len(self.scan2xy.x) > 0):
		print(self.scan2xy.x[0], self.scan2xy.y[0])

	# QUESTION:  What does this do (assuming `refFrame == 'FLU'`)?	
	# huskyWidth = 0.67	meters
	x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
	if (len(x_array) > 0):
		print(np.min(x_array))
'''	
class H1():
	
	def __init__(self):
		rospy.init_node("laser_scan_reader")
		rospy.Subscriber("/front/scan", LaserScan, self.laser_callback)
		pub = rospy.Publisher('cmd_vel', Twist,)
		rate = rospy.Rate(10)  # 10hz
		self.scan2xy = IE_tools.Scan2XY(scanTopicName   = "/front/scan", 
										refFrame        = 'FLU', 
										userAngleMinDeg = -85, 		# Far RIGHT
										userAngleMaxDeg = +85)		# Far LEFT

		while not rospy.is_shutdown():
			if obstacle_detected:
				print("Obstacle detected")
				twist = rospy.wait_for_message('cmd_vel', Twist)
				if twist.linear.x > 0:
					twist = Twist()
					twist.linear.x = 0
					pub.publish(twist)
				if twist.angular.z >0:
					pub.publish(twist)
			else:
				print("No obstacle detected")
				twist = rospy.wait_for_message('cmd_vel', Twist)
				pub.publish(twist)

		rospy.spin()

	def laser_callback(self, msg):
		global obstacle_detected
		self.scan2xy.scan2xy(msg)
		x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
		# Check if any of the laser scan points are less than the threshold
		if (np.min(x_array) < obstacle_distance_threshold):
			obstacle_detected = True
		else:
			obstacle_detected = False


if __name__ == "__main__":
	H1()
