#!/usr/bin/env python

import time
import numpy as np

from scipy.spatial.transform import Rotation as r

# disable scientific notation
np.set_printoptions(suppress=True)

class Jaco3:

    def __init__(self):

        self.joints = 7

        # kinova link lengths in meters
        d1 = -(0.1564 + 0.1284)
        d2 = -(0.0054 + 0.0064)
        d3 = -(0.2104 + 0.2104)
        d4 = -(0.0064 + 0.0064)
        d5 = -(0.2084 + 0.1059)
        d6 =   0.0
        d7 = -(0.1059 + 0.0615)
        ee = - 0.1200

        # kinova dh parameters in radians and meters
        self._dh = [[    np.pi,  0,       0,     0],
                    [np.pi/2.0,  0,      d1,     0],
                    [np.pi/2.0,  0,      d2, np.pi],
                    [np.pi/2.0,  0,      d3, np.pi],
                    [np.pi/2.0,  0,      d4, np.pi],
                    [np.pi/2.0,  0,      d5, np.pi],
                    [np.pi/2.0,  0,      d6, np.pi],
                    [    np.pi,  0, d7 + ee, np.pi]]
        
        # current configuration in radians (0th joint is always 0)
        self._q           = np.array([-1.57, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0])
        self._qdot        = np.zeros(7)
        self._qgripper    = np.array([1.0, 1.0])
        self._qdotgripper = np.array([0.1, 0.1])

        # configuration limits
        self._qLim = np.array([(np.deg2rad(-10000), np.deg2rad(10000)),
                                (np.deg2rad(-128.9), np.deg2rad(128.9)),
                                (np.deg2rad(-10000), np.deg2rad(10000)),
                                (np.deg2rad(-147.8), np.deg2rad(147.8)),
                                (np.deg2rad(-10000), np.deg2rad(10000)),
                                (np.deg2rad(-120.3), np.deg2rad(120.3)),
                                (np.deg2rad(-10000), np.deg2rad(10000))])

        self._qdotLim = np.array([(np.deg2rad(-79.64), np.deg2rad(79.64)),
                                    (np.deg2rad(-79.64), np.deg2rad(79.64)),
                                    (np.deg2rad(-79.64), np.deg2rad(79.64)),
                                    (np.deg2rad(-79.64), np.deg2rad(79.64)),
                                    (np.deg2rad(-69.91), np.deg2rad(69.91)),
                                    (np.deg2rad(-69.91), np.deg2rad(69.91)),
                                    (np.deg2rad(-69.91), np.deg2rad(69.91))])

        self._qgripperLim = np.array([(0, 1.5), (0, 1.5)])
        self._qdotgripperLim = np.array([(np.deg2rad(-0.1), np.deg2rad(0.1)),
                                            (np.deg2rad(-0.1), np.deg2rad(0.1))])

        # identity matrices
        self.identity3 = np.identity(3)
        self.identity4 = np.identity(4)
        self.identity6 = np.identity(6)

        # transformation from base to jth (i+1) frame
        self._t0j = [self.identity4, self.identity4, self.identity4,
                    self.identity4, self.identity4, self.identity4,
                    self.identity4, self.identity4]

        # transformation from ith to jth (i+1) frame
        self._tij = [self.identity4, self.identity4, self.identity4,
                    self.identity4, self.identity4, self.identity4,
                    self.identity4, self.identity4]
        
        # jacobian
        self._J  = np.zeros((6, 7))
        self._J4 = np.zeros((6, 7))

        # end-effector position and orientations
        self.position    = np.array([[0], [0], [0]])
        self.orientation = np.array([[0], [0], [0]])

        self._lastPosition    = np.array([[0], [0], [0]])
        self._lastOrientation = np.array([[0], [0], [0]])

        # timestamp
        self._timeCurrent  = time.time()
        self._timePrevious = self._timeCurrent - 0.01

        # end effector velocity
        self.linearVelocity  = np.array([[0], [0], [0]])
        self.angularVelocity = np.array([[0], [0], [0]])

        # initialize transformation matrices and Jacobian at home position
        self._updateFK()
        self._updateJacobian()
    
    # return a rotation matrix of theta in radians in X
    def _getRotationX(self, theta):
        rotation = np.array([[1, 0, 0, 0],
                            [0, np.cos(theta), -np.sin(theta), 0],
                            [0, np.sin(theta), np.cos(theta), 0],
                            [0, 0, 0, 1]])
        
        return rotation

    # return a rotation matrix of theta in radians in Y
    def _getRotationY(self, theta):
        rotation = np.array([[np.cos(theta), 0, np.sin(theta), 0],
                            [0, 1, 0, 0],
                            [-np.sin(theta), 0, np.cos(theta), 0],
                            [0, 0, 0, 1]])
        
        return rotation

    # return a rotation matrix of theta in radians in Z
    def _getRotationZ(self, theta):
        rotation = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                            [np.sin(theta), np.cos(theta), 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        
        return rotation
    
    # return a dh matrix given proper dh parameters in radians and meters
    def _getDHMatrix(self, alpha, a, d, theta):

        dh = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                        [0, np.sin(alpha), np.cos(alpha), d],
                        [0, 0, 0, 1]])
        
        return dh

    # return the position of joint i relative to frame 0
    def getPosition(self, i):

        return np.array(self._t0j[i][:3, 3])

    # return the orientation of joint i relative to frame 0 as Euler angles
    def getOrientation(self, i):

        orientation = r.from_dcm(self._t0j[i][:3,:3]).as_euler('xyz')

        return orientation

    # update the positions of joints with a list of joints
    def updateJoints(self, q):

        if len(q) != len(self._q):
            raise ValueError("Parameter 'q' is not the correct size list")
        else:
            for i in range(len(self._q)):
                # bound joints within limits
                self._q[i] = max(min(q[i], self._qLim[i][1]), self._qLim[i][0])
    
    # get joint angles
    def getJoints(self):

        return self._q
    
    # update the position of joint i
    def updateJoint(self, q, i):

        i -= 1
        if i >= len(self._q) or i < 0:
            raise ValueError("Index " + str(i) + " is out of bounds for array 'self._q'")
        else:
            # bound joint within limit
            self._q[i] = max(min(q, self._qLim[i][1]), self._qLim[i][0])
    
    # get angle of joint i
    def getJoint(self, i):

        i -= 1
        if i >= len(self._q) or i < 0:
            raise ValueError("Index " + str(i) + " is out of bounds for array 'self._q'")

        else:
            return self._q[i]
    
    # update the positions of fingers with a list of joints
    def updateFingers(self, q):

        if len(q) != len(self._qgripper):
            raise ValueError("Parameter 'qgripper' is not the correct size list")
        else:
            for i in range(len(self._qgripper)):
                # bound joints within limits
                self._qgripper[i] = max(min(q[i], self._qgripperLim[i][1]), self._qgripperLim[i][0])
    
    # update the position of finger i
    def updateFinger(self, q, i):

        i -= 1
        if i >= len(self._q) or i < 0:
            raise ValueError("Index " + str(i) + " is out of bounds for array 'self._qgripper'")
        else:
            # bound joint within limit
            self._qgripper[i] = max(min(q, self._qgripperLim[i][1]), self._qgripperLim[i][0])

    # update the joint velocities
    def updateJointVelocities(self, qdot):

        if len(qdot) != len(self._qdot):
            raise ValueError("Parameter 'qdot' is not the correct size list")
        else:
            for i in range(len(self._qdot)):
                # bound joints within limits
                self._qdot[i] = max(min(qdot[i], self._qdotLim[i][1]), self._qdotLim[i][0])
    
    # get joint velocities
    def getJointVelocities(self):

        return self._qdot
    
    # update the velocity of joint i
    def updateJointVelocity(self, qdot, i):

        i -= 1
        if i >= len(self._qdot) or i < 0:
            raise ValueError("Index " + str(i) + " is out of bounds for array 'self._qdot'")
        else:
            # bound joint within limit
            self._qdot[i] = max(min(qdot, self._qdotLim[i][1]), self._qdotLim[i][0])
    
    # get joint velocity of joint i
    def getJointVelocity(self, i):

        i -= 1
        if i >= len(self._qdot) or i < 0:
            raise ValueError("Index " + str(i) + " is out of bounds for array 'self._qdot'")
        else:
            return self._qdot[i]
    
    # update the finger velocities
    def updateFingerVelocities(self, qdot):

        if len(qdot) != len(self._qdotgripper):
            raise ValueError("Parameter 'qdotgripper' is not the correct size list")
        else:
            for i in range(len(self._qdot)):
                # bound joints within limits
                self._qdotgripper[i] = max(min(qdot[i], self._qdotgripperLim[i][1]), self._qdotgripperLim[i][0])
    
    # update the velocity of finger i
    def updateFingerVelocity(self, qdot, i):

        i -= 1
        if i >= len(self._qdotgripper) or i < 0:
            raise ValueError("Index " + str(i) + " is out of bounds for array 'self._qdotgripper'")
        else:
            # bound joint within limit
            self._qdotgripper[i] = max(min(qdot, self._qdotgripperLim[i][1]), self._qdotgripperLim[i][0])
    
    # get joint velocities given desired end-effector linear (m/s) and angular (rad/s) velocities as list or array
    def updateEEVelocity(self, pdot1, pdot2=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):

        if len(pdot1) != 6:
            raise ValueError("Invalid number of velocities in 'pdot1'")
        if len(pdot2) != 6:
            raise ValueError("Invalid number of velocities in 'pdot2'")
        
        pdot1 = np.array(pdot1)
        pdot2 = np.array(pdot2)

        # Jdls: damped least squares Jacobian
        # A = JJ^t
        # mu = sqrt(det(JJ^T))
        # Jdls = J^T(JJ^T + mu^2*I)^-1
        A = np.matmul(self._J, self._J.T)
        eigA = np.linalg.eig(A)[0]

        # different mu values to use as a damping constant
        mu1 = np.sqrt(np.max(eigA))/np.sqrt(np.min(eigA))
        mu2 = np.max(eigA)/np.min(eigA)
        mu3 = np.sqrt(np.linalg.det(A)) # larger is better

        # k1 = k(1 - m/m0)^2
        # k: constant damping factor
        # m: current manipulability
        # m0: max manipulability
        k = 0.1 # 0.01, 0.1, or 1.0 according to General Inverse Kinematics with the Error Damped Pseudoinverse
        m0 = 0.07 # taken by watching mu3 as robot moves through path
        k1 = k*(1 - mu3/m0)**2

        # print('mu:')
        # print(np.round(mu3, 3))
        # print('k:')
        # print(np.round(k1, 3))
        # print

        Jdls = np.matmul(self._J.T, np.linalg.inv(A + k1*self.identity6))

        # cartesian velocity to joint velocities
        # qdot = J(q)^-1 * pdot1
        # qdot = np.matmul(np.linalg.pinv(self._J), pdot.T)
        # qdot = np.matmul(Jdls, pdot1)

        # cartesian velocity with null space velocity
        # qdot = J^T*pdot1 + (I-J^t*J)*J^t*pdot2
        nullProject = np.eye(7) - np.matmul(np.linalg.pinv(self._J), self._J)
        qdot = np.matmul(Jdls, pdot1) + np.matmul(np.matmul(nullProject, np.linalg.pinv(self._J4)), pdot2)

        self.updateJointVelocities(qdot)

        # DEBUG
        # print(np.rad2deg(self._qdot))
        # print(pdot1)
        # print(pdot2)
        # print
    
    def simulateJointMovement(self):

        # update time
        self._timePrevious = self._timeCurrent
        self._timeCurrent  = time.time()
        dt = self._timeCurrent - self._timePrevious

        # simulate joint movement
        self.updateJoints(self._q + self._qdot * dt)
        self.updateFingers(self._qgripper + self._qdotgripper * dt)
    
    # increment robot state
    def spin(self):

        # update forward kinematics and Jacobian
        self._updateFK()
        self._updateJacobian()
    
    # build and update the jacobian matrix
    def _updateJacobian(self):

        # regular Jacobian
        Jp = [np.zeros(3), np.zeros(3), np.zeros(3),
                np.zeros(3), np.zeros(3), np.zeros(3),
                np.zeros(3)]
        Jo = [np.zeros(3), np.zeros(3), np.zeros(3),
                np.zeros(3), np.zeros(3), np.zeros(3),
                np.zeros(3)]
        
        for i in range(self.joints):
            Jo[i] = self._t0j[i][:3, 2]
            Jp[i] = np.cross(Jo[i], self._t0j[-1][:3, 3] - self._t0j[i][:3, 3])
        
        self._J = np.vstack((np.vstack(Jp).T, np.vstack(Jo).T))

        # elbow Jacobian for null space actions
        J4p = [np.zeros(3), np.zeros(3), np.zeros(3),
                np.zeros(3), np.zeros(3), np.zeros(3),
                np.zeros(3)]
        J4o = [np.zeros(3), np.zeros(3), np.zeros(3),
                np.zeros(3), np.zeros(3), np.zeros(3),
                np.zeros(3)]
        
        for i in range(3):
            J4o[i] = self._t0j[i][:3, 2]
            J4p[i] = np.cross(Jo[i], self._t0j[3][:3, 3] - self._t0j[i][:3, 3])
        
        self._J4 = np.vstack((np.vstack(J4p).T, np.vstack(J4o).T))

    # update transformation matrices and end-effector position and orientation
    def _updateFK(self):

        for i in range(len(self._dh)):
                            
            # calculate transformations from i to i+1 and 0 to i
            if i == 0:
                self._tij[i] = self._getDHMatrix(self._dh[i][0], self._dh[i][1], self._dh[i][2], \
                                                self._dh[i][3])
                self._t0j[i] = self._tij[i]
            else:
                self._tij[i] = self._getDHMatrix(self._dh[i][0], self._dh[i][1], self._dh[i][2], \
                                                self._q[i-1] + self._dh[i][3])
                self._t0j[i] = np.matmul(self._t0j[i-1], self._tij[i])
        
        # update end-effector position and orientation
        self.position    = self.getPosition(7)
        self.orientation = self.getOrientation(7)