#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
from scipy.spatial.transform import Rotation as r

from mocap_interface.msg import MocapPose
from std_msgs.msg      import Float32MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg   import ModelState

from gazebo_msgs.srv import SpawnModel, DeleteModel

# disable scientific notation
np.set_printoptions(suppress=True)

class MocapDriverNode:

    def __init__(self):

        # link lengths
        self.L1 = 1.300 # m
        # self._l1 = 0.280 # m
        self.L2 = 1.300 # m
        # self._l2 = 0.280 # m
        self.L3 = 0.080 # m

        # shoulder offset
        shoulderXOffset = -0.100
        shoulderYOffset = 0.0
        shoulderZOffset = self.L1 + self.L2/2.0

        # workspace transfer scaling factor
        userMaxReach  = np.sqrt(np.power(self.L2, 2) + 2*self.L1*self.L2) + shoulderXOffset
        print(userMaxReach)
        robotMaxReach = 0.750 # m

        self._t0 = np.array([[shoulderXOffset], [shoulderYOffset], [shoulderZOffset]])
        # self._t0 = np.array([[-0.100], [0], [0.700]]) # shoulder offset
        self._t1 = np.array([[0], [0], [-self.L1]])
        self._t2 = np.array([[0], [0], [-self.L2]])
        self._t3 = np.array([[0], [0], [-self.L3]])
        self._t4 = np.array([[0], [0], [0]])

        # need a rotation of -90 around y for each frame to match free-body diagram
        self._wf = r.from_euler('xyz', (0, -90, 0), degrees=True)

        # need a rotation to align user and robot workspaces
        # TODO: find a cleaner solution
        # self._af = r.from_euler('xyz', (-90, 0, 0), degrees=True)
        # self._af = r.from_euler('xyz', (135, 0, 0), degrees=True)
        self._af = r.from_euler('xyz', (45, 0, 0), degrees=True)


        # need a rotation to align mocap ee with robot ee
        self._ef = r.from_euler('xyz', (0, 200, 0), degrees=True) # 180 to match ee
                                                                  # 20 to account for human wrist bias

        # joint rotation matrices
        self._r0 = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        self._r1 = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        self._r2 = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        self._r3 = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        self._r4 = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

        # joint position vectors
        self._p0 = np.array([[0], [0], [0]])
        self._p1 = np.array([[0], [0], [0]])
        self._p2 = np.array([[0], [0], [0]])
        self._p3 = np.array([[0], [0], [0]])
        self._p4 = np.array([[0], [0], [0]])

        # joint quaternions
        self._q0 = [0, 0, 0, 0]
        self._q1 = [0, 0, 0, 0]
        self._q2 = [0, 0, 0, 0]
        self._q3 = [0, 0, 0, 0]
        self._q4 = [0, 0, 0, 0]

        # end-effector position and orientations
        self.position    = np.array([[0.270], [-0.230], [0.420]])
        self.orientation = np.array([[0.], [0.], [0.]])
        self.grasp       = False

        self._lastPosition    = np.array([[0.], [0.], [0.]])
        self._lastOrientation = np.array([[0], [0], [0]])

        # end effector velocity
        self.linearVelocity  = np.array([[0], [0], [0]])
        self.angularVelocity = np.array([[0], [0], [0]])

        # end effector velocity smoothing buffers
        self._bufferSize = 10
        self._linearVelocityBuffer  = []
        self._angularVelocityBuffer = []

        for _ in range(self._bufferSize):
            self._linearVelocityBuffer.append(np.zeros((3,1)))
            self._angularVelocityBuffer.append(np.zeros((3,1)))
        
        # ros spin rate
        self.rate = rospy.Rate(100)
        
        # publisher to gazebo
        self._pubGazebo   = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self._pubMocapPos = rospy.Publisher('/mocap_ee_pose_user',          MocapPose,  queue_size=10)
        
        # subscriber to imu data
        rospy.Subscriber('/imuWrist',     Vector3,    self._callbackWrist)
        rospy.Subscriber('/imuLowerArm',  Vector3,    self._callbackLowerArm)
        rospy.Subscriber('/imuUpperArm',  Vector3,    self._callbackUpperArm)
        rospy.Subscriber('/imuWristQ',    Quaternion, self._callbackWristQ)
        rospy.Subscriber('/imuLowerArmQ', Quaternion, self._callbackLowerArmQ)
        rospy.Subscriber('/imuUpperArmQ', Quaternion, self._callbackUpperArmQ)
        rospy.Subscriber('/fingersPos',   Float32MultiArray, self._callbackGlove)
    
    # return a rotation matrix of theta in X
    def _getRotationX(self, theta):
        rotation = np.array([[1, 0, 0],
                            [0, np.cos(theta), -np.sin(theta)],
                            [0, np.sin(theta), np.cos(theta)]])
        
        return rotation

    # return a rotation matrix of theta in Y
    def _getRotationY(self, theta):
        rotation = np.array([[np.cos(theta), 0, np.sin(theta)],
                            [0, 1, 0],
                            [-np.sin(theta), 0, np.cos(theta)]])
        
        return rotation

    # return a rotation matrix of theta in Z
    def _getRotationZ(self, theta):
        rotation = np.array([[np.cos(theta), -np.sin(theta), 0],
                            [np.sin(theta), np.cos(theta), 0],
                            [0, 0, 1]])
        
        return rotation

    # update joint quaternions
    def updateQ0(self, q0):
        rq0      = r.from_quat(q0)*self._af.inv()
        self._q0 = rq0.as_quat()

    def updateQ1(self, q1):
        rq1      = r.from_quat(q1)*self._af.inv()
        self._q1 = rq1.as_quat()

    def updateQ2(self, q2):
        rq2      = r.from_quat(q2)*self._af.inv()
        self._q2 = rq2.as_quat()

    def updateQ3(self, q3):
        rq3      = r.from_quat(q3)*self._af.inv()
        self._q3 = rq3.as_quat()

        # temporary fix
        self.updateQ4(q3)

    def updateQ4(self, q4):
        rq4      = self._ef*r.from_quat(q4)*self._af.inv()
        self._q4 = rq4.as_quat()

    # update joint rotation matrices
    def updateR0(self, r0):
        self._r0 = np.matmult((self._wf*self._af).as_dcm(), np.array(r0))
        self._updateFK()
    
    def updateR1(self, r1):
        self._r1 = np.matmult((self._wf*self._af).as_dcm(), np.array(r1))
        self._updateFK()
    
    def updateR2(self, r2):
        self._r2 = np.matmult((self._wf*self._af).as_dcm(), np.array(r2))
        self._updateFK()
    
    def updateR3(self, r3):
        self._r3 = np.matmult((self._wf*self._af).as_dcm(), np.array(r3))
        self._updateFK()
    
    # update joint rotation matrices by euler angles
    def updateR0Euler(self, r0):
        angles = np.radians(np.array(r0))
        rXY = np.matmul(self._getRotationX(angles[0]), self._getRotationY(angles[1]))
        self._r0 = np.matmul(rXY, self._getRotationZ(angles[2]))
        self._r0 = np.matmul((self._wf*self._af).as_dcm(), self._r0)
        
        self._updateFK()
    
    def updateR1Euler(self, r1):
        angles = np.radians(np.array(r1))
        rXY = np.matmul(self._getRotationX(angles[0]), self._getRotationY(angles[1]))
        self._r1 = np.matmul(rXY, self._getRotationZ(angles[2]))
        self._r1 = np.matmul((self._wf*self._af).as_dcm(), self._r1)
        
        self._updateFK()
    
    def updateR2Euler(self, r2):
        angles = np.radians(np.array(r2))
        rXY = np.matmul(self._getRotationX(angles[0]), self._getRotationY(angles[1]))
        self._r2 = np.matmul(rXY, self._getRotationZ(angles[2]))
        self._r2 = np.matmul((self._wf*self._af).as_dcm(), self._r2)
        
        self._updateFK()

    def updateR3Euler(self, r3):
        angles = np.radians(np.array(r3))
        rXY = np.matmul(self._getRotationX(angles[0]), self._getRotationY(angles[1]))
        self._r3 = np.matmul(rXY, self._getRotationZ(angles[2]))
        self._r3 = np.matmul((self._wf*self._af).as_dcm(), self._r3)
        
        self._updateFK()

    # update forward kinematics values
    def _updateFK(self):

        # position from base
        self._p0 = np.matmul(self._r0, self._t0)
        self._p1 = np.matmul(self._r1, self._t1) + self._p0
        self._p2 = np.matmul(self._r2, self._t2) + self._p1
        self._p3 = np.matmul(self._r3, self._t3) + self._p2
        self._p4 = np.matmul(self._r4, self._t4) + self._p3

        # orientation from base
        r01 = np.matmul(self._r0, self._r1)
        r02 = np.matmul(r01, self._r2)
        r03 = np.matmul(r02, self._r3)
        r04 = np.matmul(r03, self._r4)

        self._lastPosition    = self.position
        self._lastOrientation = self.orientation

        self.position       = self._p4
        self.orientation    = r.from_dcm(r04).as_euler('xyz')

    # publish all data to ROS and Gazebo
    def publishDataToROS(self):

        # build and publish gazebo models
        modelEE = ModelState()
        modelEE.model_name = 'mocap_frame'
        modelEE.pose.position.x    = self.position[0]
        modelEE.pose.position.y    = self.position[1]
        modelEE.pose.position.z    = self.position[2]
        modelEE.pose.orientation.w = self._q4[0]
        modelEE.pose.orientation.x = self._q4[1]
        modelEE.pose.orientation.y = self._q4[2]
        modelEE.pose.orientation.z = self._q4[3]
        # modelEE.pose.orientation.w = 0
        # modelEE.pose.orientation.x = 1
        # modelEE.pose.orientation.y = 0
        # modelEE.pose.orientation.z = 0

        # modelEEBox = ModelState()
        # modelEEBox.model_name = 'mocap_box'
        # modelEEBox.pose = modelEE.pose

        # self._pubGazebo.publish(modelEE)
        # self._pubGazebo.publish(modelEEBox)

        # # build and publish mocap pose
        mocapPose = MocapPose()
        mocapPose.pose = modelEE.pose
        mocapPose.grasp = self.grasp

        self._pubMocapPos.publish(mocapPose)


    def _callbackWrist(self, data):
        
        self.updateR3Euler([data.x, data.y, data.z])


    def _callbackLowerArm(self, data):
        
        self.updateR2Euler([data.x, data.y, data.z])

    def _callbackUpperArm(self, data):
        
        self.updateR1Euler([data.x, data.y, data.z])


    def _callbackWristQ(self, data):
        
        self.updateQ3([data.w, data.x, data.y, data.z])


    def _callbackLowerArmQ(self, data):
        
        self.updateQ2([data.w, data.x, data.y, data.z])


    def _callbackUpperArmQ(self, data):
        
        self.updateQ1([data.w, data.x, data.y, data.z])
    
    def _callbackGlove(self, dataFloat32MultiArray):

        # get average of finger values
        tot = 0
        for i in dataFloat32MultiArray.data:
            tot += i

        avg = tot/len(dataFloat32MultiArray.data)
        
        # set grasp based on finger value average
        if avg > 210:
            self.grasp = True
        else:
            self.grasp = False


def main():

    rospy.init_node('mocap_driver')
    node = MocapDriverNode()

    print('Motion Capture Driver\n')

    while not rospy.is_shutdown():
        
        node.publishDataToROS()
        node.rate.sleep()


if __name__=="__main__":
    main()
