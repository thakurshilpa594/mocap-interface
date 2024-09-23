#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from std_msgs.msg        import Float64
from control_msgs.msg    import GripperCommandActionGoal
from sensor_msgs.msg     import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# disable scientific notation
np.set_printoptions(suppress=True)

class JacoGazeboNode:

    def __init__(self):

        # joint states
        self._jointAngles     = np.zeros(7, dtype=float)
        self._jointVelocities = np.zeros(7, dtype=float)
        self._fingersPosition = 0.0

        # robot ee state
        self._eePosition         = np.zeros(3, dtype=float)
        self._eeOrientationEuler = np.zeros(3, dtype=float)

        # ros spin rate
        self.rate = rospy.Rate(100)
        
        # jaco name
        self._robotName = 'my_gen3'

        # publishers
        self._pubJoint1Pos     = rospy.Publisher('/' + self._robotName + '_gazebo/joint_1_position_controller/command', Float64, queue_size=10)
        self._pubJoint2Pos     = rospy.Publisher('/' + self._robotName + '_gazebo/joint_2_position_controller/command', Float64, queue_size=10)
        self._pubJoint3Pos     = rospy.Publisher('/' + self._robotName + '_gazebo/joint_3_position_controller/command', Float64, queue_size=10)
        self._pubJoint4Pos     = rospy.Publisher('/' + self._robotName + '_gazebo/joint_4_position_controller/command', Float64, queue_size=10)
        self._pubJoint5Pos     = rospy.Publisher('/' + self._robotName + '_gazebo/joint_5_position_controller/command', Float64, queue_size=10)
        self._pubJoint6Pos     = rospy.Publisher('/' + self._robotName + '_gazebo/joint_6_position_controller/command', Float64, queue_size=10)
        self._pubJoint7Pos     = rospy.Publisher('/' + self._robotName + '_gazebo/joint_7_position_controller/command', Float64, queue_size=10)

        self._pubFingersPos    = rospy.Publisher('/' + self._robotName + '_gazebo/robotiq_2f_85_gripper_controller/gripper_cmd/goal', 
                                                    GripperCommandActionGoal, queue_size=10)

        # subscribers
        rospy.Subscriber('/' + self._robotName + '/joint_states', JointState, self._callbackJointState)
        # rospy.Subscriber('/' + self._robotName + '_driver/in/finger_position', FingerPosition, self._callbackFingerPosition)
    
    def _callbackJointState(self, dataJointState):

        # TODO: figure out which index corresponds to which joint
        #       maybe finger positions are in here?
        jointNames = dataJointState.name

        # update joint angles
        if len(dataJointState.position) >= 7:
            self._jointAngles[0] = dataJointState.position[0]
            self._jointAngles[1] = dataJointState.position[1]
            self._jointAngles[2] = dataJointState.position[2]
            self._jointAngles[3] = dataJointState.position[3]
            self._jointAngles[4] = dataJointState.position[4]
            self._jointAngles[5] = dataJointState.position[5]
            self._jointAngles[6] = dataJointState.position[6]

        # update joint velocities
        if len(dataJointState.velocity) >= 7:
            self._jointVelocities[0] = dataJointState.velocity[0]
            self._jointVelocities[1] = dataJointState.velocity[1]
            self._jointVelocities[2] = dataJointState.velocity[2]
            self._jointVelocities[3] = dataJointState.velocity[3]
            self._jointVelocities[4] = dataJointState.velocity[4]
            self._jointVelocities[5] = dataJointState.velocity[5]
            self._jointVelocities[6] = dataJointState.velocity[6]

    # def _callbackFingerPosition(self, dataFingerPosition):

    #     # update finger positions
    #     self._fingerPositions[0] = dataFingerPosition.finger1
    #     self._fingerPositions[1] = dataFingerPosition.finger2
    
    def publishDataToROS(self):

        # publish to position controllers
        self._pubJoint1Pos.publish(Float64(self._jointAngles[0]))
        self._pubJoint2Pos.publish(Float64(self._jointAngles[1]))
        self._pubJoint3Pos.publish(Float64(self._jointAngles[2]))
        self._pubJoint4Pos.publish(Float64(self._jointAngles[3]))
        self._pubJoint5Pos.publish(Float64(self._jointAngles[4]))
        self._pubJoint6Pos.publish(Float64(self._jointAngles[5]))
        self._pubJoint7Pos.publish(Float64(self._jointAngles[6]))

        fingerGoal = GripperCommandActionGoal()
        fingerGoal.goal.command.position = self._fingersPosition
        # self._pubFingersPos.publish(fingerGoal)

def main():

    rospy.init_node('jaco_gazebo')
    node = JacoGazeboNode()

    print('Jaco Robot Gazebo Publisher\n')

    while not rospy.is_shutdown():
        
        node.publishDataToROS()
        node.rate.sleep()


if __name__ == "__main__":
    main()
