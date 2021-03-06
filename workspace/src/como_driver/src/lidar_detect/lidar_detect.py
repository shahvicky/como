#!/usr/bin/env python
'''
We get the lidar point cloud and use it to determine if there are any obstacles ahead

Author:
Sleiman Safaoui
Email:
snsafaoui@gmail.com
Github:
The-SS

Date:
Oct 3, 2018
'''


# python
from __future__ import print_function
import numpy as np
import copy
import math
from numpy import pi

# ROS
import rospy
from sensor_msgs.msg import LaserScan


class ScanSub:
	'''
	Subscribes to the lidar laser scan topic
	'''
	def __init__(self):
		self.scan_data = []
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size =1)
		
	def callback(self, data):
		self.scan_data = data
		
	def get_scan(self):
		return self.scan_data
		
		
class ScanDetect:
	'''
	Uses the obtained laser scan to determine if there are any obstacles ahead
	'''
	def __init__(self, ang_range = 40.):
		self.ang_range = ang_range #math.radians(ang_range) # range of angles to sweep in radian(about forward)
		#self.ang_min = -float(self.ang_range)/2.0 # lower bound for ang_range
		#self.ang_max = +float(self.ang_range)/2.0 # upper bound for ang_range
		self.scan = [] # scan data
		self.detected_points = [] # ranges detected in the area to scan
		self.detected_points_ang = [] # angles of points detected in the area to scan
		
	def scan_area(self, scan):
		if scan == []: # no data
			return [],[]
		self.scan = scan
		self.detected_points = [] # reset detected points
		self.detected_points_ang = [] # reset detected points
		
		if (scan.angle_min == scan.angle_max): # no lidar data
			print("Lidar data invalid")
			return [],[]
		
		if (self.ang_range > 350):
			self.detected_points = scan.ranges
			self.detected_points_ang = np.arange(0, 360, scan.angle_increment).tolist()
			return self.detected_points, self.detected_points_ang
		
		half_ang = float(self.ang_range)/2.0
		print(half_ang)
		first_half_end = 0.0 + half_ang # first half angle interval: 0 --> first_half end
		print(first_half_end)
		second_half_start = math.degrees(2 * pi) - half_ang # second half angle interval: second_half_start --> 2*PI 
		print(second_half_start)
		
		first_half_cnt = math.floor((first_half_end - 0.0) / math.degrees(scan.angle_increment)) + 1 # number of angle increments in first half
		second_half_cnt = math.floor((math.degrees(2* pi) - second_half_start) / math.degrees(scan.angle_increment)) # number of angle increments in second half
		
		if (len(scan.ranges) < (first_half_cnt + second_half_cnt)):
			print ("Invalid increment count")
			return [], []
		
		for i in range(0, int(first_half_cnt)):
			print(i)
			self.detected_points.append(scan.ranges[i])
			self.detected_points_ang.append(i * scan.angle_increment)
		for i in range(int(math.ceil(second_half_start)), int(math.ceil(second_half_start) + second_half_cnt)):
			print(i)
			print(int(math.ceil(second_half_start)))
			print(int(math.ceil(second_half_start) + second_half_cnt))
			self.detected_points.append(scan.ranges[i])
			self.detected_points_ang.append(i * scan.angle_increment)
			
		return self.detected_points, self.detected_points_ang
		
		
		'''
		
		ang_min = self.ang_min
		ang_max = self.ang_max
		if (ang_min < scan.angle_min):
			ang_min_idx = 0
			ang_min = scan.angle_min 
		else:
			ang_min_idx = math.ceil((ang_min-scan.angle_min)/scan.angle_increment) + 1 # number of increments between the lidar min. angle and the desired min. angle
			ang_min = ang_min_idx * scan.angle_increment
		if (ang_max > scan.angle_max):
			ang_max_idx = len(scan.ranges)
			ang_max = scan.angle_max
		else:
			ang_max_idx = math.floor((ang_max-scan.angle_min)/scan.angle_increment) + 1 # number of increments between the lidar min. angle and the desired max. angle
			ang_max = ang_max_idx * scan.angle_increment
			
		if ang_min_idx > ang_max_idx:
			return [],[]
		
		for i in range(int(ang_min_idx), int(ang_max_idx)+1):
			self.detected_points.append(scan.ranges[i])
			self.detected_points_ang.append(scan.angle_min + i * math.degrees(scan.angle_increment))
			
		#return self.detected_points, self.detected_points_ang
		return scan.ranges, []
		'''
		
	
class ScanPub:
	'''
	Publishes data about lidar detection
	'''
	def __init__(self):
		self.pub_data = 0.0
#		self.scan_pub = rospy.
	

def main():
	rospy.init_node("lidar_detect")
	rate = rospy.Rate(15)
	
	nodename = "/lidar_detect"
	
	old_seq = -1
	
	# Initialize nodes
	scan_sub = ScanSub()
	scan_detect = ScanDetect()
	scan_pub = ScanPub()
	
	while not rospy.is_shutdown():
		scan = scan_sub.get_scan() #get laser scan
		if (scan != []): # if scan was obtained
			if (scan.header.seq != old_seq): # new data obtained
				old_seq = scan.header.seq
				#detect using scan
				dists, angs = scan_detect.scan_area(scan)
				print ('dists', dists)
				print ('angs', angs)
		 	
				# publish result data


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException as e:
		rospy.logfatal("ROS interrupt. Shutting down lidar_detect node")
		print (e)
		pass
















