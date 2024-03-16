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
		# Translate the ROS image to a cv2 (numpy) matrix
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		# Convert the blue-green-red image to HSV
		# This will make it easier for us to filter for yellow colors
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		# Define the range of H/S/V values that match yellow:
		lower_yellow = numpy.array([ 10,  10,  10])
		upper_yellow = numpy.array([255, 255, 250])

		# `mask` will be a matrix of boolean values, indicating
		# whether each pixel is yellow or not.
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

		# We will narrow our focus to just a few lines of the image
		# that are near our robot.  
		# NOTE:  I have modified the `search_top`/`search_bot` 
		# calculation from what was in `follower_p.py`, just so you can
		# see another way to adjust the visible area.  
		h, w, d = image.shape
		search_top = int(h) - 100    # 100 pixels from the bottom of the image
		search_bot = int(h) -  60    #  60 pixels from the bottom of the image
		mask[0:search_top, 0:w] = 0  # Turn all pixels above the visible area black.
		mask[search_bot:h, 0:w] = 0  # Turn all pixels below the visible area black.

		# Overlay our mask on top of the original image:
		masked = cv2.bitwise_and(image, image, mask=mask)
		# masked[0:search_top, 0:w] = 0  # (darker gray)
		# masked[search_bot:h, 0:w] = 0  # (ligher gray)


		# -----------------------------
		# To account for multiple lines, we're going to look for 
		# multiple "blobs" of yellow pixels.
		# These "blobs" are called "contours".  To find them, it helps 
		# to do some pre-processing of the image.
		# See these links for more info:
		# - https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
		# - https://learnopencv.com/contour-detection-using-opencv-python-c/
		# -----------------------------
		# Convert our color masked image to grayscale:
		masked_gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)		
		
		# Apply "binary thresholding" to prepare the image for finding contours:
		ret, thresh = cv2.threshold(masked_gray, 150, 255, cv2.THRESH_BINARY)

		# Get our contours:
		contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		# -----------------------------

		# At this point we have (potentially) numerous "blobs" of yellow pixels.
		# We'd like to find the two largest of these blobs (ignoring random yellow pixels).
		# Then, we'll determine which blob is on the left, and which is on the right.
		# Of course, we might only find one blob, or none at all.
		
		
		# The `markers` list will store the coordinates of our left/right blobs
		markers = []

		# Find the 2 largest contours (or fewer, if there are less than 2 contours found):
		sorted_contours = self.sortContours(contours, limit=2)

		print(len(sorted_contours))
		
		# Loop over our <= 2 contours and identify the left/right ones.
		for c in sorted_contours:	
			M = cv2.moments(c)
			# M['m00'] specifies the number of pixels in the blob.
			# If there are only a few pixels, we'll assume this was noise
			# So, unless the blob has more than 6 pixels, we ignore it:
			print(M['m00'])
			if M['m00'] > 4:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				
				if (len(markers) == 0):
					# This is our first marker.  We don't know if it's left or right.
					markers.append((cx, cy))
				elif (cx > markers[0][0]):
					# This will be the right marker.  Append to end of list
					markers.append((cx, cy))
				else:
					# This is the left marker.  Insert to beginning of list
					markers.insert(0, (cx, cy))

		if (len(markers) > 1):
			# We have 2 markers.
			# Left marker is blue
			cv2.circle(masked, markers[0], 10, (255,0,0), -1)
			# Right marker is blue
			cv2.circle(masked, markers[1], 10, (0,0,255), -1)

			'''
			************** FIXME ******************
			This code was copied from follower_p.py
			Modify it to guide the robot between 
			the lines.
			***************************************			
			# Begin control based on TWO MARKERS
			err = cx - w/2
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100
			self.cmd_vel_pub.publish(self.twist)
			***************************************
			'''

		elif (len(markers) == 1):
			# Only one marker (green)
			cv2.circle(masked, markers[0], 10, (0,255,0), -1)

			'''
			************** FIXME ******************
			This code was copied from follower_p.py
			Modify it to help the robot figure out
			where the other line is.
			***************************************			
			# Begin control based on ONE MARKER
			err = cx - w/2
			self.twist.linear.x = 0.2
			self.twist.angular.z = -float(err) / 100
			self.cmd_vel_pub.publish(self.twist)
			***************************************			
			'''
		
		else:
			# There were no markers found

			'''
			************** FIXME ******************
			Add some logic here to help your robot
			find a marker.  Perhaps the robot 
			should just spin in a circle?  Maybe 
			you have a more clever idea?
			***************************************			
			'''
						
								
		# Here's where we display the masked image on the screen
		cv2.imshow("window", masked)
		cv2.waitKey(3)

	def sortContours(self, contours, limit=2):
		'''
		Returns the top `limit` number of contours, sorted by decreasing area
		'''
		sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
		return sorted_contours[0:limit]
		

rospy.init_node('follower_p_duo', anonymous=True)
camTopic = rospy.get_param("~camTopic", "/realsense/color/image_raw")
cmdVelTopic = rospy.get_param("~cmdVelTopic", "/cmd_vel")
follower = Follower()
rospy.spin()
