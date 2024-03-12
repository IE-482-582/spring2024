#!/usr/bin/env python3

'''
terminal 1: roslaunch husky_gazebo husky_playpen.launch

terminal 2: rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0 _key_timeout:=0.6 cmd_vel:=/new_cmd_vel _speed:=0.9 _turn:=0.8

terminal 3 (optional): rostopic echo /cmd_vel -p

terminal 4: python3 my_jank_node.py
'''

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import sys
sys.path.append("..")
import IE_tools.IE_tools as IE_tools
import time
from sensor_msgs.msg import LaserScan

class MyBeautifulNode ():
	def __init__(self):
		rospy.init_node('husky_teleop_relay')

		# Define subscriber for Twist messages
		rospy.Subscriber("/new_cmd_vel", Twist, self.twist_callback)

		# Define publisher for Twist messages (to Husky)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
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
		
		# Create LaserScanReader instance
		#self.laser_scan_reader = LaserScanReader()
		
		self.obstacle_threshold = 1.0  # Adjust as needed
		
		rospy.spin()
		
	def createTwistMsg(self, linearX=0, angularZ=0):
		msg = Twist()
		
		msg.linear.x  = linearX
		msg.angular.z = angularZ

		return msg
	
	def twist_callback(self, msg):
		# Check for obstacles before forwarding Twist
		x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
		if (np.min(x_array)) < self.obstacle_threshold:
			pass
		
		else:
		# If no obstacle detected or handling implemented in process_laser_scan, forward Twist
			twist = self.createTwistMsg(msg.linear.x, msg.angular.z)
			self.cmd_vel_pub.publish(twist)

	def callback_front_scan(self, msg):
		# print(msg.angle_min, msg.ranges[0])
		# Convert LaserScan data to (x,y)
		self.scan2xy.scan2xy(msg)
		'''
		# If an obstacle is detected, just print the first result:
		if (len(self.scan2xy.x) > 0):
			print(self.scan2xy.x[0], self.scan2xy.y[0])
		# QUESTION:  What does this do (assuming `refFrame == 'FLU'`)?	
		# huskyWidth = 0.67	meters
		'''
		x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
		if (len(x_array) > 0):
			print(np.min(x_array))

	def process_laser_scan(self,laser_scan_msg):
		# Extract obstacle information (adjust as needed)
		min_distance = np.min(self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))])
		
		# Check for obstacles and modify Twist message
		if min_distance < self.obstacle_threshold:
			 # Obstacle detected, apply obstacle avoidance logic
			 rospy.logwarn("Obstacle detected! Adjusting Husky's movement.")
			 # Modify twist_msg here (e.g., set to zero or adjust velocities)
		else:
			 # No obstacle, allow normal control
			 pass
		

if __name__ == '__main__':
  try:
    # Create an instance of the node class
    relay_node = MyBeautifulNode()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
