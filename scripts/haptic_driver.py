#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
import time

class HapticDriver():

	def __init__(self):
		self.forces = 0
		#self.maxRecordedForce = 0
		rospy.Subscriber("/gripper_sensors", Int16, self.forceSensorCB)
		rospy.Subscriber("/do_haptic_feedback", Bool, self.doHapticCB)
		self.do_haptics = True
		time.sleep(0.5)

	def doHapticCB(self, msg):
		self.do_haptics = msg.data
		#print(self.do_haptics)

	def forceSensorCB(self, msg):
		self.forces = msg.data
		offset = 35
		self.forces = msg.data - offset
		#print(self.forces)
		forceConstant = 12  # Load-cell calibrated constant to convert magnetic sensor values to grams.
		self.forces = forceConstant * self.forces
		#print('Gripper sensor force (grams): ' + str(self.forces))
		#if self.forces > self.maxRecordedForce:
			#self.maxRecordedForce = self.forces

	def map(self, force):
		haptic_range = [0, 255]
		force_range = [0, 1023]

		# print('Gripper Sensor Force (grams): ' + str(force))
		#print("Max Recorded Force (grams): " + str(self.maxRecordedForce))
		computation1 = ( force - force_range[0] ) * ( haptic_range[1] - haptic_range[0] )
		mapped = ( computation1 / ( force_range[1] - force_range[0] ) ) + haptic_range[0]
		mapped = int(mapped)

		if mapped < haptic_range[0]:
			mapped = haptic_range[0]
		if mapped > haptic_range[1]:
			mapped = haptic_range[1]

		# print('Haptic feedback output: ' + str(mapped))
		return mapped


if __name__ == '__main__':
	rospy.init_node('haptic_driver')
	rate = rospy.Rate(100)

	haptic_pub = rospy.Publisher("/haptic_all", Int16, queue_size=1000)
	hd = HapticDriver()

	while not rospy.is_shutdown():
		hapticFeedback = Int16()
		hapticFeedback.data = hd.map(hd.forces)
		haptic_pub.publish(hapticFeedback)

		rate.sleep()