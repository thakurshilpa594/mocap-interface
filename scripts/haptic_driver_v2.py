#!/usr/bin/env python

import roslib
import rospy

from std_msgs.msg import Int16
from std_msgs.msg import Bool
import time


class HapticDriver():

	def __init__(self):
		self.forces = 0
		rospy.Subscriber("/gripper_sensors", Int16, self.forceSensorCB)
		rospy.Subscriber("/do_haptic_feedback", Bool, self.doHapticCB)
		self.do_haptics = True
		time.sleep(0.5)

	def doHapticCB(self, msg):
		self.do_haptics = msg.data
		print(self.do_haptics)

	def forceSensorCB(self, msg):
		self.forces = msg.data
		print(self.forces)
		
	def map(self, force):
		haptic_range = [20, 255]
		force_range = [0, 130]
		
		mapped = ( ( ( force - force_range[0] ) / ( force_range[1] - force_range[0] ) ) * ( haptic_range[1] - haptic_range[0] ) ) + haptic_range[0]

		if mapped <= haptic_range[0]:
			return haptic_range[0]

		if mapped >= haptic_range[1]:
			return haptic_range[1]
		else:
			return int(mapped)

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
