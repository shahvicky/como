#!/usr/bin/env python
import roslib
import sys
import rospy
from barc.msg import ECU, Encoder
#from barc.srv import encoder_vel_calc
from como_image_processing.msg import LineData
import time
from numpy import pi
from std_msgs.msg import Float64


# TO BE MOVED TO A CONFIG FILE
heading_ctrl_Kp= 7.0
heading_ctrl_ctrl_max= 90.0
heading_ctrl_ctrl_min= -90.0

#lateral_ctrl_Kp= 438.2795
#lateral_ctrl_Ki= 15874.8968
#lateral_ctrl_Kd= 3.025

lateral_ctrl_Kp= 300.#5.2795
lateral_ctrl_Ki= 20.#.1#155.748968
lateral_ctrl_Kd= 20.#.03#3.025

lateral_ctrl_i_max= 10.0
lateral_ctrl_i_min= -10.0
lateral_ctrl_ctrl_max= 90.0
lateral_ctrl_ctrl_min= -90.0

wheel_radius = 0.0508 # in meters
# END OF THE CONFIG VARIABLES


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
		ctrl = p_ctrl + i_ctrl + d_ctrl

        # Control saturation
		ctrl = min(ctrl, self.ctrl_max)
		ctrl = max(ctrl, self.ctrl_min)

        # update previous values with current values
		self.e_sum = i_ctrl
		self.e_prev = self.e_curr
		self.t_prev = self.t_curr

		return ctrl


class LineDataFetcher:
	'''
	Fetches data published to /line_data 
	'''
	def __init__(self):
		self.data_sub = rospy.Subscriber("/line_data", LineData, self.data_callback, queue_size = 1)
		self.line_data = []

	def data_callback(self, linedata):
		self.line_data = linedata
		return

	def get_linedata(self):
		return self.line_data

		
class VelEstFetcher:
	'''
	Gets the data published to /vel_est. 
	The data is an estimate for the instanteneous velocity using the encoder readings.
	The data is calculated on the Arduino and published immediately.
	'''
	def __init__(self):
		self.data_sub = rospy.Subscriber("/vel_est", Encoder, self.data_callback, queue_size = 1)
		self.encoder_vel_data = []
	
	def data_callback(self, data):
		self.encoder_vel_data = data
		return
	
	def get_fetched_data(self):
		return self.encoder_vel_data
		
class CarVelEst:
	'''
	Estimates the velocity of the car using the vel_est data
	'''
	def __init__(self):
		self.data_curr = []
		self.data_prev = []
		self.car_speed_est = []
	
	def update_data(self, data):
		if data == []:
			return
		self.data_prev = self.data_curr
		self.data_curr = data
		return
		
	def calc_vel (self):
		if ((self.data_curr == []) | (self.data_prev == [])):
			return 0.0
		vel_sum = 0.0
		num_elts = 0.0
		if self.data_curr.FL >= 0.00001:
			vel_sum += self.data_curr.FL
			num_elts += 1.
		if self.data_curr.FR >= 0.00001:
			vel_sum += self.data_curr.FR
			num_elts += 1.
		if self.data_curr.BL >= 0.00001:
			vel_sum += self.data_curr.BL
			num_elts += 1.
		if self.data_curr.BR >= 0.00001:
			vel_sum += self.data_curr.BR
			num_elts += 1.
		if self.data_prev.FL >= 0.00001:
			vel_sum += self.data_prev.FL
			num_elts += 1.
		if self.data_prev.FR >= 0.00001:
			vel_sum += self.data_prev.FR
			num_elts += 1.
		if self.data_prev.BL >= 0.00001:
			vel_sum += self.data_prev.BL
			num_elts += 1.
		if self.data_prev.BR >= 0.00001:
			vel_sum += self.data_prev.BR
			num_elts += 1.
		if num_elts > 0:
			vel = vel_sum/num_elts
			return vel
		else:
			return 0.0
		

class OdometerEst():
	'''
	Estimates the car's traveled distance using the car's estimated speed
	'''
	def __init__(self):
		self.dist_traveled = 0.0
		self.t_prev = 0.0 # previous timestamp
		self.t_curr = 0.0 # current timestamp
		self.vel_est = 0.0 # estimated velocity between t_prev and t_curr
	
	def update_odometer(self, t, v):
		if t <= self.t_curr:
			return
		self.t_prev = self.t_curr
		self.t_curr = t
		self.vel_est = v
		dist = self.vel_est*(self.t_curr - self.t_prev)
		self.dist_traveled += dist
		return
	
	def get_odometer(self):
		return self.dist_traveled
	
		
class LineFollower:
	'''
	Finds the steering angle for the car to follow a line based on /line_data (line heading and offset of car from line)
	'''
	def __init__(self):
		self.motor = 6.5
		self.heading_ctrl = []
		self.servo = []
		self.angle_radian = [] # that should be executed
		self.angle_degree = [] # that should be executed
		self.line_pos_x = [] # that should be executed
		#self.line_pos_y = [] # that should be executed
		self.cmd_dist = [] # when to execute a command relative to the car odometery
		self.cmd_req = [] #  required data to calculate commands (indexes coupled with cmd_dist)

	def import_linedata(self, line_data): 
		#print(line_data)
		if (line_data == []):
			return
		self.angle_radian = line_data.angle_radian
		self.angle_degree = line_data.angle_degree
		self.line_pos_x = line_data.line_pos_x
		self.line_pos_y = line_data.line_pos_y
		#print(self.angle_degree, self.line_pos_x)
		
	#def import_linedata(self, line_data, traveled_dist):
	#	if (line_data == []):
	#		return
	#	l = len(self.cmd_dist) # number of entries stored in l
	#	if l == 0:
	#		self.cmd_dist.append([traveled_dist + line_data.line_pos_y])
	#		self.cmd_req.append([line_data.angle_degree, line_data.line_pos_x])
	#	else:
	#		distance_bw_data = abs(float(self.cmd_dist[-1][0]) - (traveled_dist + \
	#							line_data.line_pos_y))
	#		print('distance_bw_data', distance_bw_data)
	#		if distance_bw_data > 0.003: # distance between data is greater than a threshold
	#			self.cmd_dist.append([traveled_dist + line_data.line_pos_y])
	#			self.cmd_req.append([line_data.angle_degree, line_data.line_pos_x])
	#		# else, do not update the data (there is no need for it)
	#		print('cmd_dist', self.cmd_dist)
	#		print('cmd_req', self.cmd_req)
	#		
	#		# delete old data
	#		delete_complete = False
	#		while ((not delete_complete) & (len(self.cmd_dist) > 1)):
	#			if (float(self.cmd_dist[0][0]) - 0.7) < float(traveled_dist):
	#				del self.cmd_dist[0]
	#				del self.cmd_req[0]
	#			else:
	#				delete_complete = True
	#		# update the commands that should be executed 
	#		self.angle_degree = self.cmd_req[0][0]
	#		self.angle_radian = self.angle_degree * pi / 180.0 
	#		self.line_pos_x = self.cmd_req[0][1]
    
	def calculate_ctrl(self, heading_ctrl, lateral_ctrl, timestamp):
		if self.angle_degree == []:
			return
		if self.line_pos_x == []:
			return
		#print('Calculating ctrl')
		heading_ctrl = heading_ctrl.apply_pid(self.angle_degree, 90.0 , timestamp) # desired is angle of line, and current is always 90
		lateral_ctrl = lateral_ctrl.apply_pid(0.0, self.line_pos_x, timestamp)
		ctrl = 0.0*heading_ctrl + 1.0*lateral_ctrl
		ctrl = ctrl + 90 # add offset to remap ctrl from [-90, 90] to [0, 180]
		#print('heading_ctrl', heading_ctrl)
		#print('lateral_ctrl', lateral_ctrl)
		self.heading_ctrl = ctrl
    
	def set_servo_control(self):
		if self.heading_ctrl == []:
			self.servo = 30.0
		else:
			self.servo = self.heading_ctrl
		
	def get_ECU_data(self):
		return self.motor, self.servo

#class ECUPublisher:
#	def __init__(self):
#		self.ECU = ECU(0, 0)
#		self.ECU_pub = rospy.Publisher("/ecu", ECU, queue_size = 1)
#
#	def set_ECU(self, motor, servo):
#		self.ECU = ECU(motor, pi/180*servo)
#
#	def publish_ECU(self):
#		self.ECU_pub.publish(self.ECU)

class StrPub:
	def __init__(self):
		self.str_cmd = 0.0
		self.str_pub = rospy.Publisher("/ecu/line_follower/servo", Float64, queue_size = 1)
	
	def set_str(self, str_cmd):
		self.str_cmd = str_cmd
		
	def publish_str(self):
		self.str_pub.publish(self.str_cmd)
		

def main():
	rospy.init_node("line_follower") #initialize ros node
	rate = rospy.Rate(30)
	
	# Instantiating classes 
	fetcher = LineDataFetcher()
	#publisher = ECUPublisher()
	publisher = StrPub()
	line_follower = LineFollower()
	vel_est_fetcher = VelEstFetcher()
	car_vel_est = CarVelEst()
	heading_ctrl = PID_ctrl(Kp = heading_ctrl_Kp, Ki = 0., Kd = 0., \
							i_max = 0., i_min = 0., \
							ctrl_max = heading_ctrl_ctrl_max, ctrl_min = heading_ctrl_ctrl_min)
	lateral_ctrl = PID_ctrl(Kp = lateral_ctrl_Kp, Ki = lateral_ctrl_Ki, Kd = lateral_ctrl_Kd, \
							i_max = lateral_ctrl_i_max, i_min = lateral_ctrl_i_min, \
							ctrl_max = lateral_ctrl_ctrl_max, ctrl_min = lateral_ctrl_ctrl_min)
	odom_est = OdometerEst()
	
	# Initializations
	timestamp = time.time()
	heading_ctrl.t_prev = timestamp
	heading_ctrl.t_curr = timestamp
	lateral_ctrl.t_prev = timestamp
	lateral_ctrl.t_curr = timestamp
	odom_est.update_odometer(timestamp, 0.0)
	
	while not rospy.is_shutdown():
		timestamp = time.time()
		
		vel_est_encoders = vel_est_fetcher.get_fetched_data()
		car_vel_est.update_data(vel_est_encoders)
		car_vel = car_vel_est.calc_vel()
		odom_est.update_odometer(timestamp, car_vel)
		trav_dist = odom_est.get_odometer()
		
		line_data = fetcher.get_linedata()
		line_follower.import_linedata(line_data)#, trav_dist)
		line_follower.calculate_ctrl(heading_ctrl, lateral_ctrl, timestamp)
		line_follower.set_servo_control()

		motor, servo = line_follower.get_ECU_data()
		#publisher.set_ECU(motor, servo)
		#publisher.publish_ECU()	
		publisher.set_str(servo)
		publisher.publish_str()
	
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down line_follower node")
		pass
