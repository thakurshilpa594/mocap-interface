#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os
from mocap_interface.msg import MocapPose
from std_msgs.msg      import Float32MultiArray
import time
import csv


class MocapCalibrationNode:

	def __init__(self):
		rospy.Subscriber('/mocap_ee_pose_user',   MocapPose, self._callbackMocap)
		
		self.minRadius = []
		self.maxRadius = []
		self.minAngle = []
		self.maxAngle = []
		self.minHeight = []
		self.maxHeight = []
		self.mode = 0


	def _callbackMocap(self, msg):
		# if mode = 1:
			# measure minimum radius
		# if mode = 2:
			# measure maximum radius
		# if mode = 3:
			# measure minimum angle (to the right)
		#if mode = 4:
			# measure maximum angle (to the left)

		x = msg.pose.position.x
		y = msg.pose.position.y
		z = msg.pose.position.z

		if self.mode == 0:
			mc.minRadius.append(np.sqrt(x**2 + y**2))

		if self.mode == 1:
			mc.maxRadius.append(np.sqrt(x**2 + y**2))

		if self.mode == 2:
			mc.minAngle.append(np.arctan2(y, x))

		if self.mode == 3:
			mc.maxAngle.append(np.arctan2(y, x))

		if self.mode == 4:
			mc.minHeight.append(z)

		if self.mode == 5:
			mc.maxHeight.append(z)			


if __name__ == '__main__':

	rospy.init_node('imu_calibration_node', anonymous = True)
	rate = rospy.Rate(100)
	mc = MocapCalibrationNode()


	rospack = rospkg.RosPack()
	direct = rospack.get_path("mocap_interface")

	path = direct + '/scripts/calibration_values'

	if os.path.exists(path + "/reachCalibrations.csv"):
		os.remove(path + "/reachCalibrations.csv")
	else:
		pass

	num_values = 150

	while not rospy.is_shutdown():
		while len(mc.minRadius) <= num_values:
			mc.mode = 0
			print("Keep hand at your side", len(mc.minRadius))

		minRadius_avg = np.average(mc.minRadius)
		time.sleep(4)

		while len(mc.maxRadius) <= num_values:
			mc.mode = 1
			print("Stretch hand out", len(mc.maxRadius))

		maxRadius_avg = np.average(mc.maxRadius)
		time.sleep(4)

		while len(mc.minAngle) <= num_values:
			mc.mode = 2
			print("Hand all the way to the right", len(mc.minAngle))

		minAngle_avg = np.average(mc.minAngle)
		time.sleep(4)

		while len(mc.maxAngle) <= num_values:
			mc.mode = 3
			print("Hand all the way to the left", len(mc.maxAngle))

		maxAngle_avg = np.average(mc.maxAngle)
		time.sleep(4)

		while len(mc.minHeight) <= num_values:
			mc.mode = 4
			print("Hand all the way up", len(mc.minHeight))

		minHeight_avg = np.average(mc.minHeight)
		time.sleep(4)

		while len(mc.maxHeight) <= num_values:
			mc.mode = 5
			print("Hand all the way down", len(mc.maxHeight))

		maxHeight_avg = np.average(mc.maxHeight)

		with open(path+'/reachCalibrations.csv', 'wb') as csvfile:
			writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			writer.writerow([minRadius_avg, maxRadius_avg, minAngle_avg, maxAngle_avg, minHeight_avg, maxHeight_avg])
	
		rospy.signal_shutdown("Done Calibrating")

