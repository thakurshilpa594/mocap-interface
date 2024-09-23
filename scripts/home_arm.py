#!/usr/bin/env python

import rospy
import copy
import numpy as np

from scipy.spatial.transform import Rotation as r

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from mocap_interface.msg import MocapPose


if __name__ == '__main__':
	rospy.init_node("homeArm")
	pub = rospy.Publisher('/mocap_ee_pose', MocapPose, queue_size=10)

	while not rospy.is_shutdown():

		msg = MocapPose()

        msg.pose.position.x = 0.3
        msg.pose.position.y = 0.3
        msg.pose.position.z = 0.3

        msg.pose.orientation.x = 1
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 0

        pub.publish(msg)