#!/usr/bin/env python3

import rospy
import numpy as np

#from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist

#import IE_tools as IE_tools

import time

class MyBeautifulNode ():
	def __init__(self):
		rospy.init_node('husky_teleop_relay')

		# Define subscriber for Twist messages
		self.twist_sub = rospy.Subscriber("/cmd_vel", Twist, self.twist_callback)

		# Define publisher for Twist messages (to Husky)
		self.husky_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Timer for checking inactivity
		self.timer = rospy.Timer(rospy.Duration(0.1), self.inactivity_check)  # Calls every 0.1 seconds

		# Last received time (initialize to current time)
		self.last_cmd_time = rospy.get_time()

		# Inactivity threshold (e.g., 0.5 seconds)
		self.inactivity_timeout = 0.5

		rospy.loginfo("Husky Teleop Relay Node Started!")
		
	def twist_callback(self, data):
		# Receive Twist message from teleop_twist_keyboard
		received_twist = data
		self.last_cmd_time = rospy.get_time()  # Update last command time

		# Publish the received Twist message to the Husky controller
		self.husky_vel_pub.publish(received_twist)

	def inactivity_check(self, event):
		# Check if time since last command exceeds threshold
		if (rospy.get_time() - self.last_cmd_time) > self.inactivity_timeout:
			# Publish zero velocity if inactive
			zero_twist = Twist()
			self.husky_vel_pub.publish(zero_twist)
			rospy.loginfo("Husky stopped - No keyboard input detected!")
		
if __name__ == '__main__':
  try:
    # Create an instance of the node class
    relay_node = MyBeautifulNode()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
