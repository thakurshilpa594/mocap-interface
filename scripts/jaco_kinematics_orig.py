#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from jaco3 import Jaco3

from std_msgs.msg        import Float32
from sensor_msgs.msg     import JointState
from control_msgs.msg    import GripperCommandActionGoal
from kortex_driver.msg   import Twist
from kortex_driver.msg   import TwistCommand
from kortex_driver.msg   import JointSpeed
from kortex_driver.msg   import Base_JointSpeeds
from kortex_driver.msg   import BaseCyclic_Feedback

# disable scientific notation
np.set_printoptions(suppress=True)

class KinematicsEngineNode:

    def __init__(self):

        # initialize robot
        self._robot    = Jaco3()
        self._velocity = TwistCommand()
        self._velocityNull = TwistCommand()
        
        # ros spin rate
        self.rate = rospy.Rate(100)

        self._robotName = 'my_gen3'

        # publishers
        self._pubJointVelocities = rospy.Publisher('/' + self._robotName + '/in/joint_velocity',  Base_JointSpeeds, queue_size=10)
        #self._pubGripper         = rospy.Publisher('/' + self._robotName + '/robotiq_2f_85_gripper_controller/gripper_cmd/goal',
        #                                            GripperCommandActionGoal, queue_size=10) # range from 0.0 (open) to 0.8 (closed)
        
        # subscribers
        rospy.Subscriber('/' + self._robotName + '/in/cartesian_velocity0',     TwistCommand, self._callbackCartesianVelocity)
        rospy.Subscriber('/' + self._robotName + '/in/cartesian_velocity_null', TwistCommand, self._callbackCartesianVelocityNull)
        rospy.Subscriber('/' + self._robotName + '/joint_states',               JointState,   self._callbackJointAngles)

        # set shutdown behavior
        rospy.on_shutdown(self._onShutdown)

        # DEBUG
        self._realRobotPosition = np.zeros(3)
        self._realRobotOrientation = np.zeros(3)

        rospy.Subscriber('/' + self._robotName + '/base_feedback', BaseCyclic_Feedback, self._callbackBaseFeedback)

    # shutdown process
    def _onShutdown(self):

        # stop all joints on shutdown
        velocities = Base_JointSpeeds()

        for i in range(self._robot.joints):

            joint = JointSpeed()
            joint.joint_identifier = i
            joint.value = 0.0
            velocities.joint_speeds.append(joint)

        self._pubJointVelocities.publish(velocities)
    
    # def _callbackCommandGripper(self, dataFingerPosition):

    #     self._robot.updateFingers((dataFingerPosition.finger1, dataFingerPosition.finger2, dataFingerPosition.finger3))

    # get robot end-effector pose
    def _callbackBaseFeedback(self, dataFeedback):
        
        self._realRobotPosition = np.array((dataFeedback.base.tool_pose_x, dataFeedback.base.tool_pose_y, 
                                            dataFeedback.base.tool_pose_z))
        self._realRobotOrientation = np.array((dataFeedback.base.tool_pose_theta_x, dataFeedback.base.tool_pose_theta_y, 
                                                dataFeedback.base.tool_pose_theta_z))

    def _callbackCartesianVelocity(self, dataTwistCommand):

        # update velocity
        self._velocity = dataTwistCommand
    
    def _callbackCartesianVelocityNull(self, dataTwistCommand):

        # update null space velocity
        self._velocityNull = dataTwistCommand
    
    def _callbackJointAngles(self, dataJointState):

        # update joint angles
        self._robot.updateJoint(dataJointState.position[0], 1)
        self._robot.updateJoint(dataJointState.position[1], 2)
        self._robot.updateJoint(dataJointState.position[2], 3)
        self._robot.updateJoint(dataJointState.position[3], 4)
        self._robot.updateJoint(dataJointState.position[4], 5)
        self._robot.updateJoint(dataJointState.position[5], 6)
        self._robot.updateJoint(dataJointState.position[6], 7)
        self._robot.spin()
    
    def updateJointVelocities(self):

        # calculate joint velocities from ee velocity
        self._robot.updateEEVelocity([self._velocity.twist.linear_x, self._velocity.twist.linear_y, \
                                        self._velocity.twist.linear_z, self._velocity.twist.angular_x, \
                                        self._velocity.twist.angular_y, self._velocity.twist.angular_z],
                                        [self._velocityNull.twist.linear_x, self._velocityNull.twist.linear_y, \
                                        self._velocityNull.twist.linear_z, self._velocityNull.twist.angular_x, \
                                        self._velocityNull.twist.angular_y, self._velocityNull.twist.angular_z])
    
    def publishDataToROS(self):

        # update joint velocities
        velocities = Base_JointSpeeds()

        for i in range(self._robot.joints):

            joint                  = JointSpeed()
            joint.joint_identifier = i
            joint.value            = self._robot.getJointVelocity(i+1)

            velocities.joint_speeds.append(joint)

        self._pubJointVelocities.publish(velocities)

        # print(self._robot.position)
        # print(self._realRobotOrientation)
        # print(np.rad2deg(self._robot.orientation))
        # print(self._robot._qdot)
        # print

        # update finger positions
        # kinovaFingers = FingerPosition()
        # kinovaFingers.finger1 = self._robot._qgripper[0]
        # kinovaFingers.finger2 = self._robot._qgripper[1]
        # kinovaFingers.finger3 = self._robot._qgripper[2]

        # self._pubFingerPositions.publish(kinovaFingers)


def main():

    rospy.init_node('jaco_kinematics')
    node = KinematicsEngineNode()

    print('Jaco Kinematics Engine\n')

    while not rospy.is_shutdown():
        
        node.updateJointVelocities()
        node.publishDataToROS()
        node.rate.sleep()


if __name__ == "__main__":
    main()
