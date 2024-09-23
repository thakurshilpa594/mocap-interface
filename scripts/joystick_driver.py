#!/usr/bin/env python

import rospy
import rospkg
import time
import numpy as np

from scipy.spatial.transform import Rotation as r

from mocap_interface.msg import MocapPose
from sensor_msgs.msg     import Joy
from kortex_driver.msg   import BaseFeedback
from kortex_driver.msg   import BaseCyclic_Feedback

from gazebo_msgs.srv import SpawnModel, DeleteModel

# disable scientific notation
np.set_printoptions(suppress=True)

class MocapDriverNode:

    def __init__(self):

        self.maxLinearOffset  = 0.50 # m
        self.maxAngularOffset = np.deg2rad(70.0) # deg

        self.currentPose  = np.array([-0.269895400271, -0.23269375561, 0.619562132326, \
                                        73.66804749, -70.45474761, -7.02551])
        self.desiredPose  = np.array([-0.269895400271, -0.23269375561, 0.619562132326, \
                                        73.66804749, -70.45474761, -7.02551])
        self.offsetVector = np.zeros(6)
        self.grasp = False

        # ros spin rate
        self.rate = rospy.Rate(100)

        self._robotName = 'my_gen3'

        # publishers
        self._pubMocapPos = rospy.Publisher('/mocap_ee_pose', MocapPose,  queue_size=10)

        # subscribers
        rospy.Subscriber('/joy', Joy, self._callbackJoy)
        rospy.Subscriber('/' + self._robotName + '/base_feedback', BaseCyclic_Feedback, self._callbackBaseFeedback)

    # shutdown process
    def _onShutdown(self):

        pass

    # get robot end-effector pose
    def _callbackBaseFeedback(self, dataFeedback):
        
        # update current pose
        self.currentPose[0] = dataFeedback.base.tool_pose_x
        self.currentPose[1] = dataFeedback.base.tool_pose_y
        self.currentPose[2] = dataFeedback.base.tool_pose_z
        self.currentPose[3] = np.deg2rad(dataFeedback.base.tool_pose_theta_x)
        self.currentPose[4] = np.deg2rad(dataFeedback.base.tool_pose_theta_y)
        self.currentPose[5] = np.deg2rad(dataFeedback.base.tool_pose_theta_z)

        # modify tool pose to work with weird kinova representation
        quat = r.from_euler('xyz', self.currentPose[3:], degrees=False).as_quat()
        self.currentPose[3:] = r.from_quat(quat).as_euler('xyz', degrees=False)

    # TODO: fix control so that it sends angular velocities with respect to base frame
    def _callbackJoy(self, dataJoy):
        
        # get data from controller
        xLinear = dataJoy.axes[1]
        yLinear = dataJoy.axes[0]
        zLinear = (dataJoy.axes[2] - 1.0)/2.0 - (dataJoy.axes[5] - 1.0)/2.0
        xAngular = -dataJoy.axes[4]
        yAngular = -dataJoy.axes[3]
        zAngular = dataJoy.buttons[5] - dataJoy.buttons[4]
        
        if dataJoy.buttons[0] == 1:
            self.grasp = True
        else:
            self.grasp = False

        # determine offset vector
        offsetLinear  = np.array([xLinear, yLinear, zLinear])*self.maxLinearOffset
        offsetAngular = np.array([xAngular, yAngular, zAngular])*self.maxAngularOffset

        # build offset vector
        self.offsetVector = np.concatenate((offsetLinear, offsetAngular), axis=None)
    
    # publish all data to ROS and Gazebo
    def publishDataToROS(self):

        # add offset to current pose to get desired
        self.desiredPose = self.currentPose + self.offsetVector

        # convert orientation to quaternion
        quat = r.from_euler('xyz', self.desiredPose[3:], degrees=False).as_quat()

        # build message for mocap pose
        mocapPose = MocapPose()
        mocapPose.pose.position.x = self.desiredPose[0]
        mocapPose.pose.position.y = self.desiredPose[1]
        mocapPose.pose.position.z = self.desiredPose[2]
        mocapPose.pose.orientation.x = quat[0]
        mocapPose.pose.orientation.y = quat[1]
        mocapPose.pose.orientation.z = quat[2]
        mocapPose.pose.orientation.w = quat[3]
        mocapPose.grasp = self.grasp

        self._pubMocapPos.publish(mocapPose)

        # self._pubMocapNull.publish(mocapNull)

def main():

    rospy.init_node('joystick_driver')
    node = MocapDriverNode()

    print('Joystick Driver\n')

    while not rospy.is_shutdown():
        
        node.publishDataToROS()
        node.rate.sleep()


if __name__=="__main__":
    main()
