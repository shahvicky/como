#!/usr/bin/env python
'''
we subscribe to the steering angle topic /ecu/line_follow/str. The follower also subscribes to /april_tag/distance
and internally determines the motor command. It then publishes the ECU command with the appropriate servo values 

This script also serves as a sample script for what other COMO higher-level control scripts can look like.

Author:
Sleiman Safaoui & Kevin Daniel

Github:
The-SS

Email:
snsafaoui@gmail.com

Date:
July 30, 2018
'''

import roslib
import rospy
from barc.msg import ECU, Encoder
from como_image_processing.msg import LineData
from numpy import pi
from std_msgs.msg import Float64
import time

class PID_ctrl:
	'''
	Generic discrete time PID controller with integrator anti-windup and control output saturation
	'''

	def __init__(self, Kp = 1., Ki = 0., Kd = 0., i_max = 0., i_min = 0., ctrl_max = 1., ctrl_min = 0.):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.i_max = i_max
		self.i_min = i_min
		self.ctrl_max = ctrl_max
		self.ctrl_min = ctrl_min
		self.e_curr = 0.
		self.e_prev = 0.
		self.e_sum = 0.
		self.t_curr = 0.
		self.t_prev = 0.

	def apply_pid(self, des_value, current_value, timestamp):
		self.e_curr = des_value - current_value
		#print('error', self.e_curr)
		#print('kp', self.Kp)
		self.t_curr = timestamp
		dt = self.t_curr - self.t_prev
		#print('dt', dt)

        # Proportional control
		p_ctrl = self.Kp * self.e_curr
		#print('p_ctrl', p_ctrl)

        # Integral control with anti-windup
		i_ctrl = self.e_sum + self.Ki*dt/2.0*(self.e_curr+self.e_prev)
		i_ctrl = min(i_ctrl, self.i_max)
		i_ctrl = max(i_ctrl, self.i_min)
		#print('i_ctrl', i_ctrl)

        # Derivative control
		d_ctrl = self.Kd*(self.e_curr-self.e_prev)/dt
		#print('d_ctrl', d_ctrl)

        # Control signal calculation
		ctrl = p_ctrl #+ i_ctrl + d_ctrl
		
		#print('ctrl', ctrl)
        # Control saturation
		#print('ctrl_max', self.ctrl_max)
		#print('ctrl_min', self.ctrl_min)
		ctrl = min(ctrl, self.ctrl_max)
		#ctrl = max(ctrl, self.ctrl_min)
		#print('ctrl', ctrl)
        # update previous values with current values
		self.e_sum = i_ctrl
		self.e_prev = self.e_curr
		self.t_prev = self.t_curr

		return ctrl

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
		
class AprilTagDistSub:
	'''
	Subscribes to the april_tag/distance topic
	'''
	def __init__(self):
		self.tag_dist = 0.0
		self.dist_sub = rospy.Subscriber("/april_tag/distance", Float64, self.dist_callback, queue_size = 1)
	
	def dist_callback(self, dist):
		self.tag_dist = dist.data
	
	def get_dist(self):
		return self.tag_dist
		
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

class Platoon:
	'''
	Class for platooning, currently only written for the follower, but future work includes incorporating both leader commands as well as the ECU Publisher
	'''
	def __init__(self, leader_vel, ctrl_max, platoon_role, dist_min):
		self.motor = 0.0
		self.dist_calc = 0.0
		self.dist_calc_prev = 0.0
		self.dist_ctrl = []
		self.leader_vel = leader_vel
		self.flag = 0
		self.dist_ctrl_ctrl_max = ctrl_max
		self.platoon_role = platoon_role
		self.dist_min = dist_min

	def import_dist(self, dist):
		if (dist == -1.0):
			self.flag = 1
		else:
			self.dist_calc_prev = self.dist_calc
			self.dist_calc = dist	
		
	def calculate_ctrl(self, dist_ctrl, dist_des, timestamp):
		ctrl = dist_ctrl.apply_pid(dist_des, self.dist_calc, timestamp)
		ctrl = -1.0 * float(ctrl) + self.leader_vel #In this instance leader_vel refers to constant at which the follower travels when distance is maintained
		self.dist_ctrl = ctrl
		
	def set_motor_cmd(self):
		if (self.platoon_role == 'leader'):
			self.motor = self.leader_vel
		
		elif (self.platoon_role == 'follower'):
			if (self.flag != 1):
				if (self.dist_calc < self.dist_min): # Minimum value may need to be changed depending on testing/minimum distance tag & line is detectable
					self.motor = 0.0
				else:
					self.motor = self.dist_ctrl
			else:
				self.flag = 0
		else:
			rospy.logwarn('Invalid platoon role. Setting motor command to zero.')
		
	def get_motor_cmd(self):
		return self.motor
	
def main():
	rospy.init_node("platoon") # initialize ROS node
	rate = rospy.Rate(30)
	
	nodename = "/platoon"
	platoon_role = rospy.get_param(nodename + "/platoon_role")
	leader_vel = rospy.get_param(nodename + "/leader_vel")
	dist_des = rospy.get_param(nodename + "/dist_des")
	dist_min = rospy.get_param(nodename + "/dist_min")
	Kp = rospy.get_param(nodename + "/dist_ctrl_Kp")
	Ki = rospy.get_param(nodename + "/dist_ctrl_Ki")
	Kd = rospy.get_param(nodename + "/dist_ctrl_Kd")
	ctrl_max = rospy.get_param(nodename + "/dist_ctrl_ctrl_max")
	ctrl_min = rospy.get_param(nodename + "/dist_ctrl_ctrl_min")
	
	#Instantiating classes
	str_sub = StrSub()
	ecu_pub = ECUPub()
	dist_sub = AprilTagDistSub()
	platoon = Platoon(leader_vel, ctrl_max, platoon_role, dist_min)

	dist_ctrl = PID_ctrl(Kp, Ki, Kd, 0.0, 0.0, ctrl_max, ctrl_min)
	timestamp = time.time()
	dist_ctrl.t_prev = timestamp
	dist_ctrl.t_curr = timestamp
	
	#motor_cmd = 6.5
	while not rospy.is_shutdown():
		timestamp = time.time()
		str_cmd = str_sub.get_str() # get steering command
		
		#Find motor command
		tag_dist = dist_sub.get_dist()
		
		platoon.import_dist(tag_dist)
		platoon.calculate_ctrl(dist_ctrl, dist_des, timestamp)
		
		platoon.set_motor_cmd() #Set motor command, dependent on platoon role
		motor_cmd = platoon.get_motor_cmd()

		ecu_pub.set_ecu(motor_cmd, str_cmd) # update publisher with new command
		ecu_pub.publish_ecu() # publish new command
		
		rate.sleep()
if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down platoon_follower node")
		pass
		
