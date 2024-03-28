import rospy
import model_launcher as ml
import numpy as np

rospy.init_node('myNode', anonymous=True)

# Initialize a dictionary for all of our models/objects
objects = {}


for i in range(0,100):
	# Our "snow" will be tiny white boxes (more like falling ice cubes).
	# ----------------------------------------------------------
	# TODO: Randomize the size and origin location of each box.
	# ----------------------------------------------------------	
	dim = 0.005
	args = { 'color_r': 1.0, 'color_g': 1.0, 'color_b': 1.0,
			 'dim_x': dim, 'dim_y': dim, 'dim_z': dim, 
			 'useGravity': True}

	objects[f'box{i}'] = ml.launcher(objectType = 'box', 
						 gazeboModelName = f'box{i}',
						 namespace = 'snow', 
						 modelYawOffsetRad = 0,					  
						 x = 0, y = 0, z = 14, 
						 rollRad = 0, pitchRad = 0, yawRad = 0, 
						 args=args)	
						 
						 
