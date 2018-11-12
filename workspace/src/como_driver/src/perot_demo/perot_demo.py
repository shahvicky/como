#!/usr/bin/env python
'''
we subscribe to the steering angle topic /ecu/line_follow/str and the lidar collision flag topic /lidar/collision_flag and publish to /ecu a steering and a velocity command.

This script also serves as a sample script for what other COMO higher-level control scripts can look like.

Author:
Kevin Daniel

Github:
rev-the-kev

Date:
Oct 11, 2018
'''

import roslib
import rospy
from barc.msg import ECU, Encoder
from como_image_processing.msg import LineData
from numpy import pi
from std_msgs.msg import Float64
from std_msgs.msg import Bool


class StrSub:
	'''
	Subscribes to the steering angle topic
	'''
	def __init__(self):
		self.str_cmd = 0.0
		self.str_sub = rospy.Subscriber("/ecu/line_follower/servo", Float64, self.callback, queue_size =1)
		
	def callback(self, data):
		self.str_cmd = data.data
		
	def get_str(self):
		return self.str_cmd

class FlagSub:
	'''
	Subscribes to the LIDAR collision flag topic
	'''
	def __init__(self):
		self.collision_flag = False
		self.str_sub = rospy.Subscriber("/lidar/collision_flag", Bool, self.callback, queue_size =1)
		
	def callback(self, data):
		self.collision_flag = data.data
		
	def get_flag(self):
		return self.collision_flag

class ECUPub:
	'''
	Publishes an ECU command to /ecu topic
	'''
	def __init__(self):
		self.ecu = ECU(0.,0.)
		self.ecu_pub = rospy.Publisher('/ecu', ECU, queue_size = 1)
		
	def set_ecu(self, motor, servo):
		self.ecu = ECU(float(motor), float(servo)*pi/180.0)
	
	def publish_ecu(self):
		self.ecu_pub.publish(self.ecu)
		
def main():
	rospy.init_node("perot_demo") # initialize ROS node
	rate = rospy.Rate(30)
	str_sub = StrSub()
	ecu_pub = ECUPub()
	flag_sub = FlagSub()

	motor_cmd = 5.8  #motor_cmd = 4.5
	while not rospy.is_shutdown():
		str_cmd = str_sub.get_str() # get steering command
		
		#Check to see if collision flag is set high, if true, set motor command to zero
		collision_flag = flag_sub.get_flag()

		if collision_flag:
			ecu_pub.set_ecu(0.0, str_cmd) # update publisher with new command
		else:
			ecu_pub.set_ecu(motor_cmd, str_cmd) # update publisher with new command

		ecu_pub.publish_ecu() # publish new command
		
		rate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down line_follower_ctrl node")
		pass
		
