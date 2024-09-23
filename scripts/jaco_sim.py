#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from jaco3 import Jaco3

from sensor_msgs.msg   import JointState
from kortex_driver.msg import BaseCyclic_Feedback
from kortex_driver.msg import ActuatorFeedback
from kortex_driver.msg import JointSpeed
from kortex_driver.msg import Base_JointSpeeds

# disable scientific notation
np.set_printoptions(suppress=True)

class JacoSimNode:

    def __init__(self):

        # robot
        self._robot = Jaco3()

        # ros spin rate
        self.rate = rospy.Rate(100)
        
        # jaco type
        self._robotName = 'my_gen3'

        # publishers
        self._pubFeedback = rospy.Publisher('/' + self._robotName + '/base_feedback', BaseCyclic_Feedback, queue_size=10)
        self._pubState    = rospy.Publisher('/' + self._robotName + '/joint_states',   JointState,         queue_size=10)
        
        # subscribers
        rospy.Subscriber('/' + self._robotName + '/in/joint_velocity', Base_JointSpeeds,  self._callbackJointVelocity)
        # rospy.Subscriber('/' + self._robotName + '/in/finger_position', FingerPosition, self._callbackFingerPosition)

    def _callbackJointVelocity(self, dataBaseJointSpeeds):

        # update joint positions before new velocities are set
        self._robot.simulateJointMovement()

        # update joint velocities
        for joint in dataBaseJointSpeeds.joint_speeds:

            self._robot.updateJointVelocity(joint.value, joint.joint_identifier+1)

    # def _callbackFingerPosition(self, dataFingerPosition):

    #     # update finger positions
    #     self._robot.updateFinger(dataFingerPosition.finger1, 1)
    #     self._robot.updateFinger(dataFingerPosition.finger2, 2)
    #     self._robot.updateFinger(dataFingerPosition.finger3, 3)
    
    def spinRobot(self):

        # update robot state
        self._robot.simulateJointMovement()
        self._robot.spin()
    
    def publishDataToROS(self):

        # publish simulated robot state
        # as BaseCyclic_Feedback
        feedback = BaseCyclic_Feedback()
        feedback.base.tool_pose_x       = self._robot.position[0]
        feedback.base.tool_pose_y       = self._robot.position[1]
        feedback.base.tool_pose_z       = self._robot.position[2]
        feedback.base.tool_pose_theta_x = np.rad2deg(self._robot.orientation[0])
        feedback.base.tool_pose_theta_y = np.rad2deg(self._robot.orientation[1])
        feedback.base.tool_pose_theta_z = np.rad2deg(self._robot.orientation[2])

        for i in range(self._robot.joints):
            joint = ActuatorFeedback()
            joint.position = np.rad2deg(self._robot.getJoint(i+1))
            joint.velocity = np.rad2deg(self._robot.getJointVelocity(i+1))

            feedback.actuators.append(joint)

        self._pubFeedback.publish(feedback)

        # as JointState
        state = JointState()
        state.position = self._robot.getJoints()
        state.velocity = self._robot.getJointVelocities()

        self._pubState.publish(state)

        # DEBUG
        # print(self._robot.position)
        # print(self._robot.orientation)
        # print(self._robot.getJointVelocities())
        # print
        

def main():

    rospy.init_node('jaco_sim')
    node = JacoSimNode()

    print('Jaco Robot Simulation\n')

    while not rospy.is_shutdown():
        
        node.publishDataToROS()
        node.spinRobot()
        node.rate.sleep()


if __name__ == "__main__":
    main()