#!/usr/bin/env python

import roslib
import rospy

from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import numpy as np
import time


class HapticDriver():

	def __init__(self):
		self.forces = [0, 0, 0, 0, 0, 0]
		self.avg_force = 0
		self.max_force = 0
		self.sum_force = 0
		self.offset = 0

		self.sum_prev = 0

		rospy.Subscriber("/gripper_sensors", Float32MultiArray, self.forceSensorCB)
		rospy.Subscriber("/do_haptic_feedback", Bool, self.doHapticCB)

		time.sleep(0.5)

		self.zeroForces()

		self.do_haptics = True
	
	
	def doHapticCB(self, msg):
		self.do_haptics = msg.data
		print(self.do_haptics)

	def zeroForces(self):
		force_temps = []
		for i in range (0, 1000):
			force_temps.append(self.sum_force)

		self.offset = np.average(force_temps)

		print("Offset is ", self.offset)
		print("Len is", len(force_temps))

	def forceSensorCB(self, msg):
		self.forces = [msg.data[3], msg.data[4]]
		print(self.forces)

		#forces_right = [self.forces[0], self.forces[1], self.forces[2]]
		#forces_left = [self.forces[3], self.forces[4], self.forces[5]]
		

		self.avg_force = np.round(np.average(self.forces), 2)

		self.max_force = np.max(self.forces)

		self.sum_force = np.sum(self.forces)

		

		# self.sum_force = 0.5*self.sum_prev + 0.5*sum_force_temp

		# self.sum_prev = self.sum_force

		#print(self.forces, self.sum_force)

	def map(self, force):
		haptic_range = [30, 255]
		force_range = [60, 400]
		
		mapped = (((force - force_range[0])/(force_range[1]-force_range[0]))*(haptic_range[1]-haptic_range[0])) + float((haptic_range[0]))

		if mapped <= haptic_range[0]:
			return 0

		if mapped >= haptic_range[1]:
			return haptic_range[1]

		else:
			return mapped

if __name__ == '__main__':
	rospy.init_node('haptic_driver')
	rate = rospy.Rate(100)

	haptic_pub = rospy.Publisher("/haptic_all", Int16, queue_size=1000)
	force_sum_pub = rospy.Publisher("/force_sum", Int16, queue_size=1000)	

	hd = HapticDriver()

	while not rospy.is_shutdown():

		msg = Int16()
		msg2 = Int16()
		
		msg.data = hd.map(hd.sum_force)
		msg2.data = hd.sum_force - hd.offset

		haptic_pub.publish(msg)
		force_sum_pub.publish(msg2)
		#print(np.round(hd.sum_force, 2))

		rate.sleep()
