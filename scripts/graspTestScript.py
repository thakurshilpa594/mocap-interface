
#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
from scipy.spatial.transform import Rotation as r
from mocap_interface.msg import MocapPose
from std_msgs.msg import Int8
from std_msgs.msg import String




class GraspTester():
	def __init__(self):
		self.count = 0
		
		pose_home = np.array([0.3, 0.3, 0.3, \
						np.pi, np.pi, 0.0])

		pose_pickup = np.array([0.3, 0.3, 0.17, \
							np.pi, np.pi, 0.0])

		pose_dropoff_high = np.array([0.3, 0.1, 0.3, \
							np.pi, np.pi, 0.0])

		pose_dropoff_low = np.array([0.3, 0.1, 0.1, \
							np.pi, np.pi, 0.0])

		self.poses = [pose_home, pose_pickup, pose_home, pose_dropoff_high, pose_dropoff_low, pose_dropoff_high]

		self.desiredPose = self.poses[0]

		self.angles = r.from_quat((0, 0, 0, 1)).as_euler('xyz', degrees=False)
		
		self.pubMocapPos = rospy.Publisher('/mocap_ee_pose',          MocapPose,  queue_size=10)

		rospy.Subscriber("/key", String, self.keyboardInputCB)


	def keyboardInputCB(self, msg):
		#print("Got a CB", msg)

		if msg.data == 'a':
			self.count += 1
			self.desiredPose = self.poses[self.count % 6]
			print(self.count % 6)


if __name__ == "__main__":
	rospy.init_node('grasp_test')
	rate = rospy.Rate(100)

	gt = GraspTester()

	while not rospy.is_shutdown():

		msg = MocapPose()
		msg.pose.position.x = gt.desiredPose[0]
		msg.pose.position.y = gt.desiredPose[1]
		msg.pose.position.z = gt.desiredPose[2]
		msg.pose.orientation.w = 0
		msg.pose.orientation.x = 0
		msg.pose.orientation.y = 1
		msg.pose.orientation.z = 0

		gt.pubMocapPos.publish(msg)
		rate.sleep()

