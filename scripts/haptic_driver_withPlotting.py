#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int16
from std_msgs.msg import Bool
import time

class HapticDriver():

	def __init__(self):
		self.forces = 0
		self.maxRecordedForce = 0
		self.samples = 1000
		self.x_datapoints = np.linspace(0,self.samples,self.samples+1)
		self.y1_datapoints = np.zeros(len(self.x_datapoints))

		rospy.Subscriber("/gripper_sensors", Int16, self.forceSensorCB)
		rospy.Subscriber("/do_haptic_feedback", Bool, self.doHapticCB)
		self.do_haptics = True
		time.sleep(0.5)

	def doHapticCB(self, msg):
		self.do_haptics = msg.data
		#print(self.do_haptics)

	def update_ydata(self, y1_new):
		for i in range(len(self.y1_datapoints)-1):
			self.y1_datapoints[i] = self.y1_datapoints[i+1]
		self.y1_datapoints[-1] = y1_new

	def forceSensorCB(self, msg):
		self.forces = msg.data
		forceConstant = 15  # Load-cell calibrated constant to convert magnetic sensor values to grams.
		self.forces = forceConstant * self.forces
		#print('Gripper sensor force (grams): ' + str(self.forces))
		if self.forces > self.maxRecordedForce:
			self.maxRecordedForce = self.forces
		self.update_ydata(self.forces)

	def map(self, force):
		haptic_range = [0, 255]
		force_range = [0, 1023]

		print('Gripper Sensor Force (grams): ' + str(force))
		print("Max Recorded Force (grams): " + str(self.maxRecordedForce))
		computation1 = ( force - force_range[0] ) * ( haptic_range[1] - haptic_range[0] )
		mapped = ( computation1 / ( force_range[1] - force_range[0] ) ) + haptic_range[0]
		mapped = int(mapped)

		if mapped < haptic_range[0]:
			mapped = haptic_range[0]
		if mapped > haptic_range[1]:
			mapped = haptic_range[1]

		print('Haptic feedback output: ' + str(mapped))
		return mapped


if __name__ == '__main__':
	rospy.init_node('haptic_driver')
	rate = rospy.Rate(100)

	haptic_pub = rospy.Publisher("/haptic_all", Int16, queue_size=1000)

	plt.ion()
	fig = plt.figure()
	plt.grid(True)
	ax = fig.add_subplot(111)
	ax.set_title('Force (grams) vs. Time (seconds)', fontsize = 18)
	ax.set_xlabel('Samples in 5 seconds', fontsize = 13)
	ax.set_ylabel('Force (grams)', fontsize = 13)
	hd = HapticDriver()
	line1, = ax.plot(hd.x_datapoints, hd.y1_datapoints, 'b-')
	ax.set_ybound([0,5000])

	while not rospy.is_shutdown():
		line1.set_ydata(hd.y1_datapoints)
		fig.canvas.draw()
		fig.canvas.flush_events()
		hapticFeedback = Int16()
		hapticFeedback.data = hd.map(hd.forces)
		haptic_pub.publish(hapticFeedback)

		rate.sleep()
