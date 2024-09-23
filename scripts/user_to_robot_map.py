#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
from mocap_interface.msg import MocapPose
from gazebo_msgs.msg   import ModelState
import csv
import time

class MocapMappingNode:

	def __init__(self):
		rospy.Subscriber('/mocap_ee_pose_user',   MocapPose, self._callbackMocap)
		self._pubGazebo   = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
		self._pubMocapPos = rospy.Publisher('/mocap_ee_pose',          MocapPose,  queue_size=10)

		self.calibration_file = 'calibration_values/reachCalibrations.csv'
		self.user_minRadius = 0.0
		self.user_maxRadius = 0.0
		self.user_minAngle = 0.0
		self.user_maxAngle = 0.0
		self.user_minHeight = 0.0
		self.user_maxHeight = 0.0

		self.robot_minRadius = 0.0
		self.robot_maxRadius = 0.0
		self.robot_minAngle = 0.0
		self.robot_maxAngle = 0.0
		self.robot_minHeight = 0.0
		self.robot_maxHeight = 0.0

		self.grasp = False


		self.currentUserPos = MocapPose()

		self.getUserCalibrations()

		self.getRobotCalibrations()

		self.workspaceScale = self.robot_maxRadius / self.user_maxRadius



	def getRobotCalibrations(self):
		#minR_point = [0.126, -0.411, 0.1]
		#maxR_point = [0.474, 0.036, 0.99]

		mintheta_point = [-0.091, -0.42, 0.134]
		maxtheta_point = [0.285, 0.283, 0.113]

		minZ_point = 0.03
		maxZ_point = 0.44

		#self.robot_minRadius = np.sqrt(minR_point[0]**2 + minR_point[1]**2)
		#self.robot_maxRadius = np.sqrt(maxR_point[0]**2 + maxR_point[1]**2)

		self.robot_minRadius = 0.25
		self.robot_maxRadius = 0.75

		self.robot_minAngle = np.arctan2(mintheta_point[1], mintheta_point[0])
		self.robot_maxAngle = np.arctan2(maxtheta_point[1], maxtheta_point[0])

		self.robot_minHeight = minZ_point
		self.robot_maxHeight = maxZ_point

		print("ROBOT", self.robot_minRadius, self.robot_maxRadius, self.robot_minAngle, self.robot_maxAngle)
		print("USER", self.user_minRadius, self.user_maxRadius, self.user_minAngle, self.user_maxAngle)

	def getUserCalibrations(self):
		rospack = rospkg.RosPack()
		direct = rospack.get_path("mocap_interface")

		path = direct + '/scripts/calibration_values'

		user_values = []

		with open(path+"/reachCalibrations.csv", 'r') as f:
			reader = csv.reader(f, delimiter=',', quotechar='|', quoting=csv.QUOTE_NONNUMERIC) #This is a very brittle way of reading the csv in as floats. Eventually                                                                                               #it might be good to use pandas to do this better?     
			for row in reader:
				user_values = row

		self.user_minRadius = user_values[0]
		self.user_maxRadius = user_values[1]
		self.user_minAngle = user_values[2]
		self.user_maxAngle = user_values[3]
		self.user_maxHeight = user_values[4]
		self.user_minHeight = user_values[5]

		print(user_values)

	def map(self, num, userRange, robotRange):
		mapped = (((num - userRange[0])/(userRange[1]-userRange[0]))*(robotRange[1]-robotRange[0])) + float((robotRange[0]))

		if mapped >= robotRange[1]:
			return robotRange[1]
		elif mapped <= robotRange[0]:
			return robotRange[0]
		else:
			return mapped

	def mapUserToRobot(self):
		user_x = self.currentUserPos.pose.position.x
		user_y = self.currentUserPos.pose.position.y
		user_z = self.currentUserPos.pose.position.z

		user_R = np.abs(np.sqrt(user_x**2 + user_y**2))
		user_theta = np.arctan2(user_y, user_x)

		robot_theta = self.map(user_theta, [self.user_minAngle, self.user_maxAngle], [self.robot_minAngle, self.robot_maxAngle])
		robot_R = self.map(user_R, [self.user_minRadius, self.user_maxRadius], [self.robot_minRadius, self.robot_maxRadius])

		print("USER ", [self.user_minRadius, self.user_maxRadius], " ROBOT ",  [self.robot_minRadius, self.robot_maxRadius])

		print("USER POINT ", [user_x, user_y])
		print("RADIUS User: ", user_R, " Robot: ", robot_R)

		#print("USER ", [self.user_minAngle, self.user_maxAngle], " ROBOT ",  [self.robot_minAngle, self.robot_maxAngle])
		#print("THETA User: ", np.round(user_theta,3), " Robot: ", np.round(robot_theta,3))

		self.robot_x = robot_R * np.cos(robot_theta)
		self.robot_y = robot_R * np.sin(robot_theta)
		self.robot_z = self.map(user_z, [self.user_minHeight, self.user_maxHeight], [self.robot_minHeight, self.robot_maxHeight])
		#self.robot_z = user_z * self.workspaceScale

		#print("HEIGHT User: ", user_z, " Robot: ", self.robot_z)

	def _callbackMocap(self, msg):
		self.currentUserPos = msg

		# publish all data to ROS and Gazebo
	def publishDataToROS(self):

		# build and publish gazebo models
		modelEE = ModelState()
		modelEE.model_name = 'mocap_frame'
		modelEE.pose.position.x    = self.robot_x
		modelEE.pose.position.y    = self.robot_y
		modelEE.pose.position.z    = self.robot_z
		modelEE.pose.orientation.w = self.currentUserPos.pose.orientation.w
		modelEE.pose.orientation.x = self.currentUserPos.pose.orientation.x
		modelEE.pose.orientation.y = self.currentUserPos.pose.orientation.y
		modelEE.pose.orientation.z = self.currentUserPos.pose.orientation.z

		modelEEBox = ModelState()
		modelEEBox.model_name = 'mocap_box'
		modelEEBox.pose = modelEE.pose

		self._pubGazebo.publish(modelEE)
		self._pubGazebo.publish(modelEEBox)

		# # build and publish mocap pose
		mocapPose = MocapPose()
		mocapPose.pose = modelEE.pose
		mocapPose.grasp = self.grasp

		self._pubMocapPos.publish(mocapPose)


if __name__ == '__main__':
	rospy.init_node('mocap_mapper')
	

	mm = MocapMappingNode()
	#time.sleep(1)
	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		mm.mapUserToRobot()
		mm.publishDataToROS()
		rate.sleep()
