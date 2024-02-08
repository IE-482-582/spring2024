import numpy as np
import math
import rospy

MATH_PI_OVER_TWO  = math.pi/2.0
TWO_PI            = 2*math.pi
ONEEIGHTY_OVER_PI = 180.0/math.pi
PI_OVER_ONEEIGHTY = math.pi/180.0
PI                = math.pi


def boundedAngle(theta, angle_min, angle_max):
	'''
	Convert a given angle to be between a min/max range.
	All angles in radians.
	'''
	
	theta = theta % TWO_PI
	if theta > angle_max:
		theta -= TWO_PI
	
	if (theta < angle_min):
		raise Exception(f"Angle {theta} is outside of range [{angle_min}, {angle_max}]")        
	
	return theta		

def getIndex(theta, angle_min, angle_max, angle_increment, is_index_0_angle_max=True):
	'''
	Find the index for a given angle, theta
	
	if `is_index_0_angle_max`, then	
		angle_max corresponds to ranges[0]; 
		angle_min corresponds to ranges[-1]
		THIS IS HOW THE HUSKY'S SCANNER WORKS
	else
		angle_min corresponds to ranges[0]; 
		angle_max corresponds to ranges[-1]

	All angles in radians	
	'''
	
	try:
		theta = boundedAngle(theta, angle_min, angle_max)
		if (is_index_0_angle_max):
			# Index 0 goes with max angle
			return int((angle_max - theta)/angle_increment)
		else:
			# Index 0 goes with min angle
			return int((theta - angle_min)/angle_increment)

	except Exception as e:
		print(f'Error: {e}')


class Scan2XY():
	'''
	This class is used to convert a `sensor_msgs/LaserScan` message to (x,y) data...
	
	ASSUMPTIONS:
	* 0-degrees on the scanner is straight ahead (in the direction of +x of the vehicle).
	* (+) degrees is to the LEFT of 0, (-) degrees is to the RIGHT of 0
	 	--> `min_angle` is found if you rotate as far **clockwise** as possible
		--> `max_angle` is found if you rotate as far **counter-clockwise** as possible
		
	* If `isIndex0AngleMax` == True, 	
		the first (last) element of the `ranges` array of the LaserScan message 
	    corresponds to the scanner's `max_angle` (`min_angle`).
	    --> The **scanner** is rotating clockwise
	    THIS IS HOW OUR HUSKY WORKS.

	* If `isIndex0AngleMax` == False, 
		the first element of `ranges` corresponds to the min angle.
		--> The **scanner** is rotation ccw.
		This is NOT like Husky.
	
	`scanTopicName` is a string, with the **name** of the topic containing our LaserScan data.
	    - The topic **type** is fixed as `sensor_msgs/LaserScan`
	     
	`refFrame` Options
	'FLU'  - x Forward, y Left,  z Up       (Husky)
	'NED'  - x North,   y East,  z Down     (quadcopters)
	'ENU'  - x East,    y North, z Up       (Cartesian, plotting)
	'Opt'  - x right,   y down,  z forward  (optical frame)
	'''

	# Let the user specify limits on what our scanner looks at.
	# This is helpful if the scanner itself is blocked 
	# by other parts of the robot.
	USER_ANGLE_MIN_DEG = -85    # (max rotation to the RIGHT)
	USER_ANGLE_MAX_DEG = +85    # (max rotation to the LEFT)

	IS_INDEX_0_ANGLE_MAX = True  # (see note above)

	TIMEOUT_SEC = 10  # [seconds].  How long to wait for init laserscan message in Scan2XY


	def __init__(self, scanTopicName, 
				 isIndex0AngleMax = IS_INDEX_0_ANGLE_MAX, 
				 refFrame         = 'ENU', 
				 userAngleMinDeg  = USER_ANGLE_MIN_DEG, 
				 userAngleMaxDeg  = USER_ANGLE_MAX_DEG, 
				 timeoutSec       = TIMEOUT_SEC):
				 
		# Just grab 1 message from our LaserScan topic so we can
		# use some properties of the scanner.
		# NOTE:  This does not **subscribe** us to the topic.
		from sensor_msgs.msg import LaserScan		
		msg = rospy.wait_for_message(scanTopicName, LaserScan, timeoutSec)
			
		self.setProperties(userAngleMinDeg, userAngleMaxDeg, isIndex0AngleMax, refFrame, msg)
		# This should set self.index_min, self.index_max, self.xconv_array, self.yconv_array

		# Initialize our array of (x,y) detections.
		# The LaserScan callback will need to call the `scan2xy()` function 
		# to updates these values.
		self.x = np.array([])
		self.y = np.array([])
	

	def setConversionArray(self, refFrame, scan_angles):
		# FIXME -- This depends on our reference frame:
		if (refFrame == 'FLU'):
			# +x is forward, +y is left.  Like Husky drives
			self.xconv_array =  np.cos(scan_angles)
			self.yconv_array =  np.sin(scan_angles)
			
		elif (refFrame == 'NED'):
			# +x is North, +y is East.  Like quadcopters.
			self.xconv_array =  np.cos(scan_angles)
			self.yconv_array = -np.sin(scan_angles)

		elif (refFrame == 'ENU'):
			# +x is East, +y is North.  Like Cartesian, plotting
			self.xconv_array = -np.sin(scan_angles)
			self.yconv_array =  np.cos(scan_angles)

		elif (refFrame == 'Opt'):
			# +x is right, +y is down.  Like drawing on HTML canvas
			self.xconv_array = -np.sin(scan_angles)
			self.yconv_array =  np.cos(scan_angles)
		
		else:		
			raise Exception(f'ERROR: Unknown refFrame {refFrame}.')

		
	def setProperties(self, userAngleMinDeg, userAngleMaxDeg, isIndex0AngleMax, refFrame, msg):
		'''
		user angles start at far right and increase COUNTER-clockwise.
		`isIndex0AngleMax` refers to the relationship between the `ranges` list and 
		the direction that the scanner itself rotates.   
		
		msg is a LaserScan message
		'''
		# Convert the user's angle limits from deg to rad:
		userAngleMinRad = np.deg2rad(userAngleMinDeg)  # Far RIGHT
		userAngleMaxRad = np.deg2rad(userAngleMaxDeg)  # Far LEFT
		
		# These are all of the angles picked up by our scanner, filtered for the subset we care about.
		if (isIndex0AngleMax):
			# ranges[0] is from MAX angle.
			scan_angles = np.arange(userAngleMaxRad, userAngleMinRad, -msg.angle_increment)
			tmpAngle    = userAngleMaxRad
		else:
			# ranges[0] is from MIN angle
			scan_angles = np.arange(userAngleMinRad, userAngleMaxRad, msg.angle_increment)
			tmpAngle    = userAngleMinRad
					
		# Find the indices associated with the user's angle limits.
		# tmpAngle is found above. 
		self.index_min = getIndex(tmpAngle, 
								  msg.angle_min, msg.angle_max, 
								  msg.angle_increment, isIndex0AngleMax)
		
		self.index_max = self.index_min + len(scan_angles)

		# Get 2 arrays: self.xconv_array and self.yconv_array
		# These arrays will be used to convert LaserScan to XY
		# The conversion depends on the chosen reference frame
		self.setConversionArray(refFrame, scan_angles)
			
		# FIXME -- Revisit this
		# self.scan_conv  = np.dstack([self.xconv_array, self.yconv_array])[0]
		# This results in an nx2 array (where n is the number of angles, and 2 represents x,y)
		# NOT USING THIS FOR NOW.  BUT COULD USE THIS TO PUBLISH TO POINTCLOUD TOPIC

		# Not sure if we want/need to save these for later: 
		self.angle_min       = msg.angle_min
		self.angle_max       = msg.angle_max
		self.angle_increment = msg.angle_increment
		self.range_min       = msg.range_min
		self.range_max       = msg.range_max


	def scan2xy(self, msg):
		'''
		msg is a LaserScan message
		
		Sets the values of the x and y arrays
		'''
		ranges = np.array(msg.ranges[self.index_min: self.index_max])
		mask = np.isfinite(ranges)
		
		self.x = ranges[mask]*self.xconv_array[mask]
		self.y = ranges[mask]*self.yconv_array[mask]
