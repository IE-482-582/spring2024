#!/usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist

import sys, os
sys.path.append(f"{os.environ['HOME']}/Projects/IE-482-582/spring2024/Projects/IE_tools")
import IE_tools as IE_tools

import time

# ----------------------------------------
CMD_VEL_RATE = 10  # [Hz]

EPSILON = 0.1          # [meters].  Tolerance to reaching goal destination

MAX_LINEAR_X           = 1.0   # [m/s]
SLOWING_FACTOR_LINEAR  = 0.8   # unitless

MAX_ANGULAR_Z          = 0.05  # [rad/s]
SLOWING_FACTOR_ANGULAR = 0.5   # unitless
		
TWO_PI = 2*np.math.pi
# ----------------------------------------
	
	
class Zamboni():
	def __init__(self):
	
		rospy.init_node("zamboni_driver", anonymous=True)
	
		rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)	

		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.rate = rospy.Rate(CMD_VEL_RATE)	# [Hz]

		# Set the shutdown function
		rospy.on_shutdown(self.shutdown)
		
		# Initialize these as None until we get real values in the odom callback:
		self.pos_body_x_m = None
		self.pos_body_y_m = None
		self.heading      = None		
		
		while ((self.pos_body_x_m is None) or (self.pos_body_y_m is None) or  (self.heading is None)):
			print('waiting for the odom callback to provide real values for these variables...')
			time.sleep(1)   # We use the Python `sleep` function here because we don't care about a specific run rate.

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
		
	def getStatus(self, xCur, yCur, xGoal, yGoal, hdgDeg):
		'''
		FIXME -- Document what on earth this function actually does?!!
		TA - take input of current XY and goal XY and heading in degrees
		returns the distance to the target and the angle/direction to target
		'''
		distance = IE_tools.dist2target(xCur, yCur, xGoal, yGoal)
		
		(angleDeg, sign) = IE_tools.getLocalHeadingDeg(xCur, yCur, xGoal, yGoal, hdgDeg)

		return (distance, angleDeg, sign)
				
	def pController(self, dist2target, angleDeg, sign):
		angleRad = np.deg2rad(angleDeg)
		
		print(dist2target, angleDeg, sign)
		linearX = min(MAX_LINEAR_X, dist2target/(1/CMD_VEL_RATE)*SLOWING_FACTOR_LINEAR)		# [m/s]
		
		if(CMD_VEL_RATE<1):
			angleRadAbs = angleRad*CMD_VEL_RATE*SLOWING_FACTOR_ANGULAR
		else:
			angleRadAbs = angleRad*SLOWING_FACTOR_ANGULAR
			angleRadAbs = min(MAX_ANGULAR_Z,angleRadAbs)
		
		angularZ = angleRadAbs*sign
			
		# FIXME -- Write the proportional controller here...
		# angularZ <= MAX_ANGULAR_Z	
		# we have `angleDeg` and `sign` available
		# CAUTION:  angleDeg is in DEGREES...angularZ MUST BE IN RADIANS PER SECOND
			
		return (linearX, angularZ)
		
	def run(self):
		'''
		This is the function that will publish Twist commands
		'''
		
		# Get our goal destination, where do we go first
		#get (xGoal, yGoal)
		(xGoal, yGoal) = (10,0)
		
		while not rospy.is_shutdown():
			# What is our status?
			#update goal 
			(dist2goal, angleDeg, sign) = self.getStatus(self.pos_body_x_m, 
													   self.pos_body_y_m, 
													   xGoal, yGoal, 
													   self.heading)
			 
			# Have we reached the goal?
			if (dist2goal < EPSILON):
				print("hello world!")
				'''
				get the next goal
				if there's no next goal:
					break
				'''
			else:
				# find out how to control the robot
				(linearX, angularZ) = self.pController(dist2goal, angleDeg, sign)
				
				# publish Twist command		
				twistMsg = self.createTwistMsg(linearX, angularZ)						
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
		
if __name__ == "__main__":
	Zamboni()
