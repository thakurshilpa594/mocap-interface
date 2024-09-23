#!/usr/bin/env python

import roslib
import rospy
import time
import numpy as np

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray


class GripperSensorFilter():
	def __init__(self):
		rospy.init_node('gripper_sensor_filter')
		self.rate = rospy.Rate(100)
		
		rospy.Subscriber("/gripper_sensors_raw", Float32MultiArray, self.gripperSensorCB)
		self.pub_sensor = rospy.Publisher("/gripper_sensors", Float32MultiArray, queue_size=1000)

		self.offsets = [0, 0, 0, 0, 0, 0]

		self.previous_forces = [0, 0, 0, 0, 0, 0]
		self.filtered_forces = [0, 0, 0, 0, 0, 0]

		time.sleep(1)

		self.calibrate()

	def calibrate(self):
		force_temps = [[], [], [], [], [], []]
		
		for i in range (0, 1000):
			for j in range(0, len(self.filtered_forces)):
				force_temps[j].append(self.filtered_forces[j])

		for i in range(0, len(force_temps)):
			self.offsets[i] = np.average(force_temps[i])

		print("Offsets are ", self.offsets)
		print("Len is", len(force_temps))

	def gripperSensorCB(self, msg):
		
		current_readings = [a_i - b_i for a_i, b_i in zip(msg.data, self.offsets)]			
		#self.filtered_forces = msg.data

		for i in range(0, len(current_readings)):
			self.filtered_forces[i] = np.round((0.6 * self.previous_forces[i] + 0.4 * current_readings[i]), 2)
			self.previous_forces[i] = self.filtered_forces[i]

		print(self.filtered_forces)


if __name__ == '__main__':
	gsf = GripperSensorFilter()

	while not rospy.is_shutdown():
		msg = Float32MultiArray()
		msg.data = gsf.filtered_forces

		gsf.pub_sensor.publish(msg)

		gsf.rate.sleep()
