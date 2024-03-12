#!/usr/bin/env python3

"""
How to run:

Terminal 1: (open gazebo)


Terminal 2:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Terminal 3:
python3 safe_teleop.py
"""

import numpy as np
import math
import sys
import os
import time

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#sys.path.append(f"{os.environ['HOME']}/Projects/IE-482-582/spring2024/Projects/IE_tools")
import IE_tools as IE_tools

"""
Assumptions:
    -teleop_twist_keyboard is called using the parameter cmd_vel:=teleop_cmd_vel
        Ex. rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0 _key_timeout:=0.6 cmd_vel:=teleop_vel _speed:=0.9 _turn:=0.8
    -The only Twist commands that will be received are linear.x > 0 and angular.z. All other components are 0.
    -params['FRONT_CLEARANCE'] and params['SIDE_CLEARANCE'] are the MINIMUM clearances that will
     be used. Based on the robot's speed and turning speed, the command may require extra clearance.
     The only exception, is less clearance than params['SIDE_CLEARANCE'] may be used when turning
     in the opposite direction.
    -The FLU reference frame is used.
    -The first LaserScan data is available prior to the first Teleop command.
"""


class SafeTeleop():
    def __init__(self, params, constants):
        self.params = params
        self.constants = constants

        rospy.init_node("safe_teleop", anonymous=True)

        # Initialize a converter from LaserScan to XY
        # Use use 'FLU' which means +x is 'F'orward, +y is 'L'eft and +z is 'U'p.
        self.scan2xy = IE_tools.Scan2XY(scanTopicName="/front/scan",
                                        refFrame='FLU',
                                        userAngleMinDeg=-1 * self.params['LASER_SCAN_MAX_ANGLE'],  # Far RIGHT
                                        userAngleMaxDeg=self.params['LASER_SCAN_MAX_ANGLE'])  # Far LEFT

        # rospy.Subscriber("<topic name>", <topic type>, <callback>)
        rospy.Subscriber("/front/scan", LaserScan, self.callback_front_scan)
        rospy.Subscriber(self.params['INPUT_NAME'], Twist, self.callback_teleop)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(self.params['CMD_VEL_RATE'])  # [Hz]

        # Set the shutdown function
        rospy.on_shutdown(self.shutdown)

        rospy.spin()  # Keep node open while waiting for messages

    def forward_distance_danger(self, msg):
        # TODO: combine forward and lateral, instead of separate
        # Speed factor: Increase forward distance by a factor proportional to the robot's linear speed
        speed_factor = max(1, self.params['SPEED_CLEARANCE_FACTOR'] * abs(msg.linear.x))  # Ensure at least a factor of 1

        # Adjusting forward distance based on the speed factor
        dist = self.params['FRONT_CLEARANCE'] * speed_factor

        return dist

    def lateral_distance_danger(self, msg):
        """
        Finds the distance to the left and right that we care about.
        If an obstacle is within this range, we will deny the movement command.
        :return: left_dist, right_dist
            These are the distances to the left and right, resp., that
            would be dangerous to the robot if there was an obstacle
            in that region.
        """
        base_clearance = self.params['SIDE_CLEARANCE']
        husky_width = self.constants['HUSKY_WIDTH']

        # Speed factor: dynamically adjust lateral clearance based on speed
        speed_factor = max(1, self.params['SPEED_CLEARANCE_FACTOR'] * abs(msg.linear.x))

        # Base factors for turning, ensuring at least a value of 1
        turn_magnitude = max(1, abs(msg.angular.z))

        # Calculate dynamic factors based on turning direction and magnitude
        if msg.angular.z > 0:  # Turning left
            # Increase left distance danger zone based on turn speed
            left_dist_factor = self.params['TURNING_CLEARANCE_FACTOR'] * turn_magnitude
            # Decrease right distance, but ensure it doesn't go too low
            right_dist_factor = max(self.params['TURNING_CLEARANCE_REDUCTION'], 1 / turn_magnitude)
        elif msg.angular.z < 0:  # Turning right
            # Decrease left distance based on turn speed
            left_dist_factor = max(self.params['TURNING_CLEARANCE_REDUCTION'], 1 / turn_magnitude)
            # Increase right distance danger zone
            right_dist_factor = self.params['TURNING_CLEARANCE_FACTOR'] * turn_magnitude
        else:  # Moving straight, keep factors balanced
            left_dist_factor = right_dist_factor = 1

        # Apply the factors to the base clearance, adjusted for speed
        left_dist = (base_clearance + 0.5 * husky_width) * speed_factor * left_dist_factor
        right_dist = (base_clearance + 0.5 * husky_width) * speed_factor * right_dist_factor

        return left_dist, right_dist

    def callback_front_scan(self, msg):
        # Convert LaserScan data to (x,y)
        self.scan2xy.scan2xy(msg)

    def callback_teleop(self, msg):
        if msg.linear.x > self.params['MAX_LINEAR_SPEED']:
            msg.linear.x = self.params['MAX_LINEAR_SPEED']
            print("Too fast. We are limiting your speed.")

        # Implement your logic here to determine if the message is safe
        is_safe = self.check_if_safe(msg)
        if is_safe:
            if msg.linear.x > 0 or abs(msg.angular.z) > 0:
                print("Command is safe.")

            # If the message is safe, publish it to the robot
            self.cmd_vel_pub.publish(msg)
        else:
            print("Unsafe Twist command received; command blocked.")

    def check_if_safe(self, msg):
        # Always safe if only turning
        if msg.linear.x == 0 and msg.linear.y == 0:
            return True

        # Find the coordinates of obstacles within a dangerous forward distance.
        x_danger_mask = self.scan2xy.x < self.forward_distance_danger(msg)

        # Find the coordinates of obstacles within a dangerous lateral distance.
        left_dist, right_dist = self.lateral_distance_danger(msg)
        # Both left_dist and right_dist should be positive, but the absolute value is used for safety.
        y_danger_mask = ((self.scan2xy.y < abs(left_dist)) &
                         (self.scan2xy.y > -1 * abs(right_dist)))

        danger_mask = x_danger_mask & y_danger_mask

        # Find the points in the danger region.
        x_array = self.scan2xy.x[danger_mask]
        y_array = self.scan2xy.y[danger_mask]

        # If there are no data entries in the danger region, then the path is clear.
        # These two checks are probably redundant. We probably only need one of these.
        return len(x_array) == 0 and len(y_array) == 0

    def shutdown(self):
        print("The safe_teleop node is shutting down now.")
        twistMsg = Twist()
        self.cmd_vel_pub.publish(twistMsg)


if __name__ == "__main__":
    params = {
        'INPUT_NAME': 'teleop_vel',  # This is the name we give to the cmd_vel messages sent by the teleoperator
        'CMD_VEL_RATE': 10,  # (hz)

        'FRONT_CLEARANCE': 1.0,  # (meters)
        'SIDE_CLEARANCE': 0.50,  # (meters)
        'SPEED_CLEARANCE_FACTOR': 2.5,  # (unitless) Larger values leads to more aggressive filtering at high speeds.
        'TURNING_CLEARANCE_FACTOR': 1.5,  # (unitless) Larger values leads to more aggressive filtering at high turning speeds.
        'TURNING_CLEARANCE_REDUCTION': 0.5,  # (unitless) Smaller values allow less clearance on the side opposite of turning.

        'MAX_LINEAR_SPEED': 0.75,  # (m/s)

        'LASER_SCAN_MAX_ANGLE': 85  # (degrees)

    }

    constants = {
        # Husky dimensions: https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/
        'HUSKY_WIDTH': 0.670,  # (meters)
        'HUSKY_LENGTH': 0.990  # (meters)
    }

    SafeTeleop(params=params, constants=constants)
