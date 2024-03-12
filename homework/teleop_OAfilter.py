#!/usr/bin/env python3

#how to run the code: 
#cd to the directory containing the python file
#open another terminal and run this:
#rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0 _key_timeout:=0.6 cmd_vel:=my_cmd_vel _speed:=0.9 _turn:=0.8
#keep this terminal open for the keyboard control


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import sys,os
sys.path.append(f"{os.environ['HOME']}/Projects/IE-482-582/spring2024/Projects/IE_tools")
import IE_tools as IE_tools
CMD_VEL_RATE = 15 


class OAfilter():
    def __init__(self):
        
        rospy.init_node("teleop_OAfilter", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.Subscriber("my_cmd_vel", Twist, self.callback_input_vel)	
        rospy.Subscriber("/front/scan", LaserScan, self.callback_front_scan)

        self.rate = rospy.Rate(CMD_VEL_RATE)	# [Hz]
        self.linXoutput = 0
        self.angZoutput = 0
        self.angZinput = 0
        self.linXinput = 0
        self.obs_dist = 100 #distance to obstacle

        # Initialize a converter from LaserScan to XY
        self.scan2xy = IE_tools.Scan2XY(scanTopicName   = "/front/scan", 
                                        refFrame        = 'FLU', 
                                        userAngleMinDeg = -85, 		# Far RIGHT
                                        userAngleMaxDeg = +85)		# Far LEFT        
        self.publish()
        rospy.spin()
        pass

    def callback_input_vel(self,msg):
        self.linXinput = msg.linear.x
        self.angZinput = msg.angular.z
        self.publish()
        pass

    def createTwistMsg(self, linearX=0, angularZ=0):
        msg = Twist()
        msg.linear.x  = linearX
        msg.angular.z = angularZ
        return msg


    def callback_front_scan(self, msg):

        # Convert LaserScan data to (x,y)
        self.scan2xy.scan2xy(msg)
        x_array = self.scan2xy.x[((self.scan2xy.y < +0.5) & (self.scan2xy.y > -0.5))]
        if (len(x_array) > 0):
            print(np.min(x_array))
            self.obs_dist = np.min(x_array)
        

    def publish(self):
        #while not rospy.is_shutdown():
        twistMsg = self.createTwistMsg(self.linXinput, self.angZinput)						
        if self.obs_dist <= 2:
            twistMsg = self.createTwistMsg(0, self.angZinput)
        self.cmd_vel_pub.publish(twistMsg)
            #self.rate.sleep()
        pass 


if __name__ == "__main__":
	OAfilter()
