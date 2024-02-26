import numpy as np
import math
import rospy
# import geopy.distance

MATH_PI_OVER_TWO  = math.pi/2.0
TWO_PI            = 2*math.pi
ONEEIGHTY_OVER_PI = 180.0/math.pi
PI_OVER_ONEEIGHTY = math.pi/180.0
PI                = math.pi



def dist2target(xCur, yCur, xGoal, yGoal):
	'''
	Return the distance from the current location to the goal location
	'''
	return math.sqrt((xGoal-xCur)**2 + (yGoal-yCur)**2)


def getAngleBetwPtsRad(xCur, yCur, xGoal, yGoal):
	'''
	What is the angle, in [radians], from (xCur, yCur) to (xGoal, yGoal)?
	
	NOTES:
	* The reference frame is global/fixed. 
	    - This is NOT relative to the orientation of a robot.
	* The 0-radian angle is along the "x" direction.
	    - This works for North/East/Down, Forward/Left/Up, or Cartesian coordinate systems,
	      as long as the right-hand rule is followed.
	'''
	return (math.atan2(yGoal-yCur, xGoal-xCur) + TWO_PI) % (TWO_PI)  # In the range [0,2*pi]
	
def getAngleBetwPtsDeg(xCur, yCur, xGoal, yGoal):
	'''
	What is the angle, in [degrees], from (xCur, yCur) to (xGoal, yGoal)?	
	'''
	return np.rad2deg(getAngleBetwPtsRad(xCur, yCur, xGoal, yGoal))  # In the range [0,360]

def getLocalHeadingRad(xCur, yCur, xGoal, yGoal, hdgRad):
	'''
	Find the angle, in [radians], required to move from current location to goal location, 
	relative to the robot's current heading (hdgRad).
	
	NOTES:
	* This function returns 2 values:
		1. The rotation, in [radians], required to face the target. 
		    - This is relative to the current orientation of the robot.
		    - The 0-radian angle is in the direction the robot is facing. 
		2. Either +1 to indicate that the rotation should be in the positive direction, or 
		          -1 to indicate negative direction.

	For example, suppose we have a husky, which uses the Forward/Left/Up coordinate system.
	Its rotation is positive in the CCW direction.
	
	If our function returns `(.3, -1)`, this means that we should rotate .3 radians in the CW direction.
	'''
	# First, find the global angle from current to goal:
	globalAngleRad = getAngleBetwPtsRad(xCur, yCur, xGoal, yGoal)
	
	# Now, find the difference from current heading, in range [0, 2Pi]:
	localAngleRad = boundedAngle(globalAngleRad - hdgRad, 0, TWO_PI)
	
	# Now, find the shortest angle and direction of rotation to reach this goal:
	return shortestRotationRad(0, localAngleRad)

	
def getLocalHeadingDeg(xCur, yCur, xGoal, yGoal, hdgDeg):
	'''
	See description for `getLocalHeadingRad`
	'''
	(angleRad, sign) = getLocalHeadingRad(xCur, yCur, xGoal, yGoal, np.deg2rad(hdgDeg))
	 
	return (np.rad2deg(angleRad), sign)
	
	
def shortestRotationRad(thetaCurrent, thetaGoal):
	# What is the shortest angle to traverse to get from thetaCurrent to thetaGoal?
	# 	thetaCurrent in [0, 2Pi]
	# 	thetaGoal in [0, 2Pi]
	thetaCurrent = boundedAngle(thetaCurrent, 0, TWO_PI)
	thetaGoal    = boundedAngle(thetaGoal,    0, TWO_PI)

	ccwAngle = (thetaCurrent-thetaGoal)%TWO_PI
	cwAngle  = (thetaGoal-thetaCurrent)%TWO_PI
	if (ccwAngle < cwAngle):
		return (ccwAngle, -1)  # angle in the range [0,2*pi]
	else:
		return (cwAngle, +1)   # angle in the range [0,2*pi]
		
def shortestRotationDeg(thetaCurrent, thetaGoal):
	# What is the shortest angle to traverse to get from thetaCurrent to thetaGoal?
	# 	thetaCurrent in [0, 360]
	# 	thetaGoal in [0, 360]

	(angleRad, sign) = shortestRotationRad(np.deg2rad(thetaCurrent), np.deg2rad(thetaGoal))
	return (np.rad2deg(angleRad), sign)
			
	
	
def getPointInDistXfwdRad(xCur, yCur, hdgRad, distMeters):
	"""
	Generate [x, y] coordinate given a current coordinate, a direction, and a distance.

	NOTE: +x is forward.
	If using NED coordinate system, 
		+x is forward, +y is right, +z is down
		0 radians is forward, and increases CLOCKWISE (e.g., PI/2 degrees is EAST)
	If using FLU coordinate system,
		+x is forward, +y is left, +z is up
		0 radians is forward, and increases COUNTERCLOCKWISE (e.g., PI/2 degrees is WEST)

	Parameters
	----------
	xCur, yCur: floats
		Current location
	direction: float
		The direction, range [0, 2*Pi] in radians, 0 is in direction of +x
	distMeters: float
		The distance between current point and our destination
	Returns
	-------
	list
		A location in distance with given direction, in [x, y] form.
	"""
	x = xCur + distMeters * math.cos(hdgRad)
	y = yCur + distMeters * math.sin(hdgRad)

	return [x, y]			

def getPointInDistXfwdDeg(xCur, yCur, hdgDeg, distMeters):
	
	return getPointInDistXfwdRad(xCur, yCur, np.deg2rad(hdgDeg), distMeters)

"""
We might want/need this function in the future.
The idea is to find the location of a point that is [x, y] meters away from our
current location, given that we are currently facing hdgDeg degrees in a world frame.
	
def geoPointInDistanceXY(loc, x, y, hdgDeg):
	'''
	Given
		loc [lat, lon]
		x   [meters]
		y   [meters]
		hdg world frame, 0 north, 90 east
	Return
		[lat, lon]
	
	'''
	
	dist = math.sqrt(x**2 + y**2)
	bearing = effHeadingDeg(hdgDeg, x, y)
	return list(geopy.distance.distance(meters=dist).destination(point=loc, bearing=bearing))[0:2]
"""
	
	
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
