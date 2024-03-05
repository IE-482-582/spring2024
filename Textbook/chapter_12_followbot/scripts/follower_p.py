#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber(camTopic, 
										  Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher(cmdVelTopic,
										   Twist, queue_size=1)
		self.twist = Twist()

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([ 10,  10,  10])
		upper_yellow = numpy.array([255, 255, 250])
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
			
		h, w, d = image.shape
		search_top = int(3*h/4)
		search_bot = int(3*h/4) + 20
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0

		# Just for visualizing what's happening:
		masked = cv2.bitwise_and(image, image, mask=mask)
		masked[0:search_top, 0:w] = 100  # (darker gray)
		masked[search_bot:h, 0:w] = 200  # (ligher gray)

		M = cv2.moments(mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(masked, (cx, cy), 20, (0,0,255), -1)
			
			# Begin control
			err = cx - w/2
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100
			self.cmd_vel_pub.publish(self.twist)

		cv2.imshow("window", masked)
		cv2.waitKey(3)

rospy.init_node('follower_p', anonymous=True)
camTopic = rospy.get_param("~camTopic", "/realsense/color/image_raw")
cmdVelTopic = rospy.get_param("~cmdVelTopic", "/cmd_vel")
follower = Follower()
rospy.spin()
