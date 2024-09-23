#!/usr/bin/env python

import rospy
import copy
import numpy as np
#Requires older version of numpy, 1.16.0
import fcl

from scipy.spatial.transform import Rotation as r

from std_msgs.msg        import Float32
from sensor_msgs.msg     import JointState
from geometry_msgs.msg   import Pose
from geometry_msgs.msg   import PoseStamped
from mocap_interface.msg import MocapPose
from mocap_interface.msg import Prop
from mocap_interface.msg import Props
from kortex_driver.msg   import Twist
from kortex_driver.msg   import TwistCommand
from kortex_driver.msg   import BaseFeedback
from kortex_driver.msg   import BaseCyclic_Feedback

from jaco3 import Jaco3

# disable scientific notation
np.set_printoptions(suppress=True)

class PathPlannerNode():

    def __init__(self):

        # autonomy features
        self.BOUNDARYAVOIDANCE = True
        self.OBSTACLEAVOIDANCE = True
        self.SNAPTOOBJECT      = True

        # [x, y, z, wx, wy, wz]
        self.errorEE              = np.zeros(6)
        self.errorNull            = np.zeros(6)
        self.currentPose          = np.array([0.3, 0.3, 0.3, \
                                                np.pi, 0.0, 0.0])
        self.desiredPose          = np.array([0.3, 0.3, 0.3, \
                                                np.pi, 0.0, 0.0])
        self.mocapPose            = np.array([0.3, 0.3, 0.3, \
                                                np.pi, 0.0, 0.0])

        # object avoidance/attraction thresholds (m)
        self._avoidInnerThreshold = 0.02
        self._avoidOuterThreshold = 0.14

        self._attractOuterThreshold = 0.10
        self._attractInnerThreshold = -0.015

        self._modVelocityLinear = 1.0 # %

        # robot model to keep track of transformations and forward kinematics
        self._simRobot = Jaco3()
        
        # messages to build and send
        self.robotTwistCommand     = TwistCommand()
        self.robotNullTwistCommand = TwistCommand()

        # max ee velocity is 50 cm/s
        self.MAXSPEEDLINEAR = 0.5
        self.MAXSPEEDANGULAR = np.deg2rad(70)

        # speed reduction threshold
        self.speedReductionThresholdLinear  = 0.20
        self.speedReductionThresholdAngular = np.deg2rad(30)

        # current speeds
        self._speedLinearEE = 0.0 # m/s

        # important props: {'name': (prop, [collision], snap)}
        self._props = {}

        # add boundary collision objects (fcl.Plane is broken so using boxes while continuously updating their positions)
        boundX = fcl.Box(0.05, 0.30, 0.30)
        boundY = fcl.Box(0.30, 0.05, 0.30)
        boundZ = fcl.Box(0.30, 0.30, 0.05)
        boundP = fcl.Box(0.05, 0.20, 0.30)

        tf = fcl.Transform()

        propXHi      = Prop()
        propXHi.name = 'o_boundxhi'
        propXLo      = Prop()
        propXLo.name = 'o_boundxlo'
        propYHi      = Prop()
        propYHi.name = 'o_boundyhi'
        propYLo      = Prop()
        propYLo.name = 'o_boundylo'
        propZHi      = Prop()
        propZHi.name = 'o_boundzhi'
        propZLo      = Prop()
        propZLo.name = 'o_boundzlo'
        propPHi      = Prop()
        propPHi.name = 'o_boundphi'
        propPLo      = Prop()
        propPLo.name = 'o_boundplo'

        self._props['o_boundxhi'] = (propXHi, [fcl.CollisionObject(boundX, tf)], fcl.CollisionObject(boundX, tf))
        self._props['o_boundxlo'] = (propXLo, [fcl.CollisionObject(boundX, tf)], fcl.CollisionObject(boundX, tf))
        self._props['o_boundyhi'] = (propYHi, [fcl.CollisionObject(boundY, tf)], fcl.CollisionObject(boundY, tf))
        self._props['o_boundylo'] = (propYLo, [fcl.CollisionObject(boundY, tf)], fcl.CollisionObject(boundY, tf))
        self._props['o_boundzhi'] = (propZHi, [fcl.CollisionObject(boundZ, tf)], fcl.CollisionObject(boundZ, tf))
        self._props['o_boundzlo'] = (propZLo, [fcl.CollisionObject(boundZ, tf)], fcl.CollisionObject(boundZ, tf))
        self._props['o_boundphi'] = (propPHi, [fcl.CollisionObject(boundP, tf)], fcl.CollisionObject(boundP, tf))
        self._props['o_boundplo'] = (propPLo, [fcl.CollisionObject(boundP, tf)], fcl.CollisionObject(boundP, tf))

        # workspace restrictions (m)
        # bounds take into account the sizes of boundary collision objects
        self._wsBoundaryX = (-0.025, 1.025)
        self._wsBoundaryY = (-1.025, 1.025)
        self._wsBoundaryZ = (-0.102, 1.025) # robot mounting plate is 22mm tall
                                            # also an offset of 55mm to balance boundary avoidance
        self._wsBoundaryP = ( 0.055, 0.825)

        # important poses
        self._robotJointPoses = [Pose(), Pose(), Pose(), Pose(), Pose(), Pose()]
        self._robotPose       = Pose()
        self._mocapPose       = Pose()
        self._nextPose        = Pose()

        # important states
        self._isGrasping   = False

        # ros spin rate
        self.rate = rospy.Rate(100)

        self._robotName = 'my_gen3'

        # publishers
        self.pubKinova     = rospy.Publisher('/' + self._robotName + '/in/cartesian_velocity0',     TwistCommand, queue_size=10)
        self.pubKinovaNull = rospy.Publisher('/' + self._robotName + '/in/cartesian_velocity_null', TwistCommand, queue_size=10)

        # subscribers
        rospy.Subscriber('/props',         Props,       self._callbackProps)
        rospy.Subscriber('/mocap_ee_pose', MocapPose,   self._callbackMocapPose)
        rospy.Subscriber('/' + self._robotName + '/base_feedback', BaseCyclic_Feedback, self._callbackBaseFeedback)
        rospy.Subscriber('/' + self._robotName + '/joint_states',  JointState,          self._callbackJointAngles)
        
        # set shutdown behavior
        rospy.on_shutdown(self._onShutdown)
    
    def _onShutdown(self):

        stopVel = TwistCommand()
        stopVel.twist.linear_x = 0.0
        stopVel.twist.linear_y = 0.0
        stopVel.twist.linear_z = 0.0
        stopVel.twist.angular_x = 0.0
        stopVel.twist.angular_y = 0.0
        stopVel.twist.angular_z = 0.0

        self.pubKinova.publish(stopVel)
    
    # determine attracting or repelling velocity
    def getPotentialFieldVelocity(self):

        velocityEE   = np.zeros(3)
        velocityNull = np.zeros(3)
        joints       = self._robotJointPoses + [self._robotPose]

        closestDistanceEE   = 10.0 # m
        closestDistanceNull = 10.0 # m

        closestSpeedEE   = 0.0 # m/s
        closestSpeedNull = 0.0 # m/s

        jointObject  = fcl.Sphere(0.050)
        eeObject     = fcl.Box(0.155, 0.035, 0.070)
        
        # find closest obstacle for each velocity space
        for i in range(len(joints)):

            # create collision object for each joint or end effector
            if i < len(joints) - 1:
                jointTF      = fcl.Transform(np.array([joints[i].position.x, joints[i].position.y, joints[i].position.z]))
                jointCObject = fcl.CollisionObject(jointObject, jointTF)

            else:
                jointTF      = fcl.Transform(
                                np.array([joints[i].orientation.w, joints[i].orientation.x, joints[i].orientation.y, joints[i].orientation.z]),
                                np.array([joints[i].position.x, joints[i].position.y, joints[i].position.z]))
                jointCObject = fcl.CollisionObject(eeObject, jointTF)

            for name, prop in self._props.iteritems():

                # determine if avoidance is necessary
                tok = prop[0].name.split('_')

                #             obstacle         container
                if (tok[0] == 'o' and self.OBSTACLEAVOIDANCE) or ('bound' in tok[1] and self.BOUNDARYAVOIDANCE):

                    for propCObject in prop[1]:

                        # find the closest distance based on collision geometries
                        req = fcl.DistanceRequest()
                        res = fcl.DistanceResult() # res.nearest_points is innacurate and a known issue
                        distanceFCL = max(fcl.distance(jointCObject, propCObject, req, res), 0.0001)

                        # find the direction based on cobject transforms
                        propPosition  = propCObject.getTranslation()
                        jointPosition = np.array((joints[i].position.x, joints[i].position.y, joints[i].position.z))
                        difference    = propPosition - jointPosition
                        distance      = max(np.linalg.norm(difference), 0.0001)
                        direction     = difference/distance
                        
                        if distanceFCL <= self._avoidOuterThreshold:
                                
                            # calculate avoidance speed
                            speed = 0.0 # m/s

                            if distanceFCL <= self._avoidInnerThreshold:
                                # speed = -self._speedLinearEE
                                speed = -self._speedLinearEE*1.2
                                # speed = self._speedLinearEE*(1.0 + (self._avoidInnerThreshold - distance)/ \
                                #             self._avoidInnerThreshold)
                            
                            else:
                                speed = -(self._avoidOuterThreshold - distanceFCL)/ \
                                            (self._avoidOuterThreshold - self._avoidInnerThreshold)*(self._speedLinearEE*1.2)
                            
                            # add to avoidance velocity
                            # full velocity is summed to properly weight each avoidance direction
                            velocityEE += direction*speed

                            # update closest object
                            if distanceFCL < closestDistanceEE:
                                closestDistanceEE = copy.deepcopy(distanceFCL)
                                closestSpeedEE    = copy.deepcopy(speed)
                            
                            if i <= 2:
                                velocityNull += direction*min(speed*3, self.MAXSPEEDLINEAR)

                                # update closest object
                                if distanceFCL < closestDistanceNull:
                                    closestDistanceNull = copy.deepcopy(distanceFCL)
                                    closestSpeedNull    = copy.deepcopy(speed)

        # calculate avoidance directions and apply respective speeds to get velocities
        magEE   = max(np.linalg.norm(velocityEE), 0.0001)
        magNull = max(np.linalg.norm(velocityNull[:2]), 0.0001)

        velocityEE   = np.concatenate((-(velocityEE/magEE)*closestSpeedEE, np.zeros(3)), axis=None)
        velocityNull = np.concatenate((-(velocityNull[:2]/magNull)*closestSpeedNull, np.zeros(4)), axis=None)

        # scale down velocity of everything when close to an obstacle (between 100% and 50%)
        self._modVelocityLinear = min(1.0, max(0.5, 1.0 - (self._avoidOuterThreshold - min(closestDistanceEE, closestDistanceNull))/ \
                                                            (self._avoidOuterThreshold - self._avoidInnerThreshold)))

        return velocityEE, velocityNull

    # determine object to snap to
    def snapToObject(self):

        closestMocap     = 10.0 # m
        closestCollision = 10.0 # m
        closestProp      = np.zeros(2)
        closestDirection = np.zeros(2)

        # create geometry for mocap snapping
        mocapObject = fcl.Sphere(0.005)
        
        # find closest graspable to desired location
        for name, prop in self._props.iteritems():

            tok = prop[0].name.split('_')

            if tok[0] == 'p' or tok[0] == 'g':

                # put together transform for mocap at snap geometry height
                t  = np.array([self._mocapPose.position.x, self._mocapPose.position.y, prop[0].pose.position.z])
                tf = fcl.Transform(t)
                mocapCObject = fcl.CollisionObject(mocapObject, tf)

                # find the closest distance based on collision geometries
                req = fcl.DistanceRequest()
                res = fcl.DistanceResult() # res.nearest_points is innacurate and a known issue
                distFCL = max(fcl.distance(mocapCObject, prop[2], req, res), 0.0001)
                
                # figure out distance and direction to graspable in 2D space
                propPosition  = np.array((prop[0].pose.position.x, prop[0].pose.position.y))
                mocapPosition = np.array((self._mocapPose.position.x, self._mocapPose.position.y))
                diffMocap     = propPosition - mocapPosition

                distMocap = np.linalg.norm(diffMocap) if np.linalg.norm(diffMocap) else 0.0001
                dirMocap  = diffMocap/distMocap

                if distMocap < closestMocap:
                    
                    closestMocap     = distMocap
                    closestCollision = distFCL
                    closestProp      = propPosition
                    closestDirection = dirMocap
        
        # update desired pose with graspable radius if within threshold
        if closestCollision <= self._attractOuterThreshold and closestCollision > 0.0001:

            self.desiredPose[0] = closestProp[0] - closestDirection[0]*(closestMocap - closestCollision + self._attractInnerThreshold)
            self.desiredPose[1] = closestProp[1] - closestDirection[1]*(closestMocap - closestCollision + self._attractInnerThreshold)
    
    # update model joint angles
    def _callbackJointAngles(self, dataJointState):

        # update joint angles
        self._simRobot.updateJoint(dataJointState.position[0], 1)
        self._simRobot.updateJoint(dataJointState.position[1], 2)
        self._simRobot.updateJoint(dataJointState.position[2], 3)
        self._simRobot.updateJoint(dataJointState.position[3], 4)
        self._simRobot.updateJoint(dataJointState.position[4], 5)
        self._simRobot.updateJoint(dataJointState.position[5], 6)
        self._simRobot.updateJoint(dataJointState.position[6], 7)
        self._simRobot.spin()

        j6 = self._simRobot.getPosition(6)
        j6Pose = Pose()
        j6Pose.position.x = j6[0]
        j6Pose.position.y = j6[1]
        j6Pose.position.z = j6[2]

        j5 = self._simRobot.getPosition(5)
        j5Pose = Pose()
        j5Pose.position.x = j5[0]
        j5Pose.position.y = j5[1]
        j5Pose.position.z = j5[2]

        j4 = self._simRobot.getPosition(4)
        j4Pose = Pose()
        j4Pose.position.x = j4[0]
        j4Pose.position.y = j4[1]
        j4Pose.position.z = j4[2]

        j3 = self._simRobot.getPosition(3)
        j3Pose = Pose()
        j3Pose.position.x = j3[0]
        j3Pose.position.y = j3[1]
        j3Pose.position.z = j3[2]

        # add intermediate point between j4 and j5
        difference    = j5 - j4
        distance  = np.linalg.norm(difference)
        direction = difference/distance

        j4a = j4 + direction*(distance/2.0)
        j4aPose = Pose()
        j4aPose.position.x = j4a[0]
        j4aPose.position.y = j4a[1]
        j4aPose.position.z = j4a[2]

        self._robotJointPoses = [j3Pose, j4Pose, j4aPose, j5Pose, j6Pose]

    # get all props
    def _callbackProps(self, dataProps):

        for prop in dataProps.props:

            # put together transform
            t  = np.array([prop.pose.position.x, prop.pose.position.y, prop.pose.position.z])
            q  = np.array([prop.pose.orientation.w, prop.pose.orientation.x, prop.pose.orientation.y, prop.pose.orientation.z])
            tf = fcl.Transform(q, t)

            # parse name to determine geometry
            tok = prop.name.split('_')

            if tok[1] == 'box1':
                obj = fcl.Box(0.203, 0.206, 0.293)
                collision = [fcl.CollisionObject(obj, tf)]
                snap      = fcl.CollisionObject(obj, tf)

            elif tok[1] == 'block2in':
                obj = fcl.Box(0.049, 0.049, 0.049)
                collision = [fcl.CollisionObject(obj, tf)]
                snap      = fcl.CollisionObject(obj, tf)

            elif tok[1] == 'bin':
                obj = fcl.Box(0.220, 0.130, 0.125)
                collision = [fcl.CollisionObject(obj, tf)]
                snap      = fcl.CollisionObject(obj, tf)
            
            elif tok[1] == 'post':
                # obj = fcl.Cylinder(0.015, 0.600)
                obj = fcl.Cylinder(0.015, 0.050)
                collision = []

                # offsets for long obstacle
                off = np.array([0.0, 0.0, 0.050])

                for i in range(-8, 9):
                    tfo = fcl.Transform(q, t + i*off)
                    collision.append(fcl.CollisionObject(obj, tfo))
                
                snap = fcl.CollisionObject(obj, tf)

            self._props[prop.name] = (prop, collision, snap)

    # get mocap end-effector position
    def _callbackMocapPose(self, dataMocapPose):

        self._mocapPose = dataMocapPose.pose

        try:
            angles = r.from_quat((dataMocapPose.pose.orientation.x, dataMocapPose.pose.orientation.y, dataMocapPose.pose.orientation.z, \
                                    dataMocapPose.pose.orientation.w)).as_euler('xyz', degrees=False)
        except:
            angles = r.from_quat((0, 0, 0, 1)).as_euler('xyz', degrees=False)
        
        # update mocap pose
        self.mocapPose[0] = dataMocapPose.pose.position.x
        self.mocapPose[1] = dataMocapPose.pose.position.y
        self.mocapPose[2] = dataMocapPose.pose.position.z
        self.mocapPose[3] = angles[0]
        self.mocapPose[4] = angles[1]
        self.mocapPose[5] = angles[2]

        self._isGrasping = dataMocapPose.grasp
        
        # set destination pose
        self.desiredPose = self.mocapPose
        
        # snap to nearest object if close enough
        if self.SNAPTOOBJECT:
            self.snapToObject()
    
    # get next significant pose
    def _callbackNextPose(self, dataPose):
        
        self._nextPose = dataPose

    # get robot end-effector pose
    def _callbackBaseFeedback(self, dataFeedback):
        
        self._robotPose.position.x    = dataFeedback.base.tool_pose_x
        self._robotPose.position.y    = dataFeedback.base.tool_pose_y
        self._robotPose.position.z    = dataFeedback.base.tool_pose_z

        quat = r.from_euler('xyz', (np.deg2rad(dataFeedback.base.tool_pose_theta_x),
                            np.deg2rad(dataFeedback.base.tool_pose_theta_y),
                            np.deg2rad(dataFeedback.base.tool_pose_theta_z))).as_quat()
        self._robotPose.orientation.x = quat[0]
        self._robotPose.orientation.y = quat[1]
        self._robotPose.orientation.z = quat[2]
        self._robotPose.orientation.w = quat[3]
        
        # update current pose
        self.currentPose[0] = dataFeedback.base.tool_pose_x
        self.currentPose[1] = dataFeedback.base.tool_pose_y
        self.currentPose[2] = dataFeedback.base.tool_pose_z
        self.currentPose[3] = np.deg2rad(dataFeedback.base.tool_pose_theta_x)
        self.currentPose[4] = np.deg2rad(dataFeedback.base.tool_pose_theta_y)
        self.currentPose[5] = np.deg2rad(dataFeedback.base.tool_pose_theta_z)

        # modify tool pose to work with weird kinova representation
        self.currentPose[3:] = r.from_quat(quat).as_euler('xyz', degrees=False)

        # update boundary obstacles
        self._props['o_boundxhi'][0].pose.position.y = self.currentPose[1]
        self._props['o_boundxhi'][0].pose.position.z = self.currentPose[2]
        self._props['o_boundxhi'][1][0].setTransform(fcl.Transform((self._wsBoundaryX[1], self.currentPose[1], self.currentPose[2])))

        self._props['o_boundxlo'][0].pose.position.y = self.currentPose[1]
        self._props['o_boundxlo'][0].pose.position.z = self.currentPose[2]
        self._props['o_boundxlo'][1][0].setTransform(fcl.Transform((self._wsBoundaryX[0], self.currentPose[1], self.currentPose[2])))

        self._props['o_boundyhi'][0].pose.position.x = self.currentPose[0]
        self._props['o_boundyhi'][0].pose.position.z = self.currentPose[2]
        self._props['o_boundyhi'][1][0].setTransform(fcl.Transform((self.currentPose[0], self._wsBoundaryY[1], self.currentPose[2])))

        self._props['o_boundylo'][0].pose.position.x = self.currentPose[0]
        self._props['o_boundylo'][0].pose.position.z = self.currentPose[2]
        self._props['o_boundylo'][1][0].setTransform(fcl.Transform((self.currentPose[0], self._wsBoundaryY[0], self.currentPose[2])))

        self._props['o_boundzhi'][0].pose.position.x = self.currentPose[0]
        self._props['o_boundzhi'][0].pose.position.y = self.currentPose[1]
        self._props['o_boundzhi'][1][0].setTransform(fcl.Transform((self.currentPose[0], self.currentPose[1], self._wsBoundaryZ[1])))

        self._props['o_boundzlo'][0].pose.position.x = self.currentPose[0]
        self._props['o_boundzlo'][0].pose.position.y = self.currentPose[1]
        self._props['o_boundzlo'][1][0].setTransform(fcl.Transform((self.currentPose[0], self.currentPose[1], self._wsBoundaryZ[0])))

        # get direction using vector base to ee
        dir          = self.currentPose[:2]/np.linalg.norm(self.currentPose[:2])
        boundPolarHi = dir*self._wsBoundaryP[1]
        boundPolarLo = dir*self._wsBoundaryP[0]

        # get quaternion rotation in z
        quatPolar = r.from_euler('xyz', (0.0, 0.0, np.arctan2(dir[1], dir[0]))).as_quat()
        quatPolar = np.array((quatPolar[3], quatPolar[0], quatPolar[1], quatPolar[2]))

        self._props['o_boundphi'][0].pose.position.x = boundPolarHi[0]
        self._props['o_boundphi'][0].pose.position.y = boundPolarHi[1]
        self._props['o_boundphi'][0].pose.position.z = self.currentPose[2]
        self._props['o_boundphi'][1][0].setTransform(fcl.Transform(quatPolar, (boundPolarHi[0], boundPolarHi[1], self.currentPose[2])))

        self._props['o_boundplo'][0].pose.position.x = boundPolarLo[0]
        self._props['o_boundplo'][0].pose.position.y = boundPolarLo[1]
        self._props['o_boundplo'][0].pose.position.z = self.currentPose[2]
        self._props['o_boundplo'][1][0].setTransform(fcl.Transform(quatPolar, (boundPolarLo[0], boundPolarLo[1], self.currentPose[2])))
    
    # find error between current and desired poses
    def updateError(self):

        # find linear error
        linearError = self.desiredPose[:3] - self.currentPose[:3]

        # find angular error
        rotDesired = r.from_euler('xyz', self.desiredPose[3:], degrees=False)
        rotCurrent = r.from_euler('xyz', self.currentPose[3:], degrees=False)
        angularError = (rotDesired*rotCurrent.inv()).as_euler('xyz', degrees=False)

        # calculate point to pull j4 upwards
        # errorJ4  = np.array((self.currentPose[0]/2.0, self.currentPose[1]/2.0, self._wsBoundaryZ[1]*2.0))

        self.errorEE   = np.concatenate((linearError, angularError), axis=None)
        self.errorNull = np.concatenate((np.array((0.0, 0.0, self.MAXSPEEDLINEAR)), np.zeros(3)))

    # find the desired velocity to reduce error
    def getErrorVelocity(self):
        
        # set linear velocity
        magLinearEE = np.linalg.norm(self.errorEE[:3])
        if magLinearEE == 0:
            magLinearEE = 0.00001

        dirLinearEE = self.errorEE[:3]/magLinearEE

        # max speed until distance threshold reached
        if magLinearEE > self.speedReductionThresholdLinear:
            self._speedLinearEE = self.MAXSPEEDLINEAR

        # then scale down speed
        else:
            self._speedLinearEE = self.MAXSPEEDLINEAR*(magLinearEE/self.speedReductionThresholdLinear)

        # account for rotation around -180 and 180
        for i in range(3, 6):
            if self.errorEE[i] >= np.pi:
                self.errorEE[i] -= 2.0*np.pi
            elif self.errorEE[i] < -np.pi:
                self.errorEE[i] += 2.0*np.pi
        
        # set angular velocity
        magAngularEE = np.linalg.norm(self.errorEE[3:])
        if magAngularEE == 0:
            magAngularEE = 0.00001

        dirAngularEE = self.errorEE[3:]/magAngularEE

        # max speed until angle treshold reached
        if magAngularEE > self.speedReductionThresholdAngular:
            speedAngularEE = self.MAXSPEEDANGULAR
        
        # then scale down speed
        else:
            speedAngularEE = self.MAXSPEEDANGULAR*(magAngularEE/self.speedReductionThresholdAngular)

        # build velocity
        linearVelocityEE  = np.array((dirLinearEE[0]*self._speedLinearEE, dirLinearEE[1]*self._speedLinearEE,
                                        dirLinearEE[2]*self._speedLinearEE))
        angularVelocityEE = np.array((dirAngularEE[0]*speedAngularEE, dirAngularEE[1]*speedAngularEE,
                                        dirAngularEE[2]*speedAngularEE))
        
        velocityEE = np.concatenate((linearVelocityEE, angularVelocityEE), axis=None)

        # calculate null space velocity
        magLinearNull = np.linalg.norm(self.errorNull[:3])
        if magLinearNull == 0:
            magLinearNull = 0.00001

        dirLinearNull = self.errorNull[:3]/magLinearNull

        # no need to scale down speed on approach
        linearVelocityNull = np.array((dirLinearNull[0]*self.MAXSPEEDLINEAR, dirLinearNull[1]*self.MAXSPEEDLINEAR,
                                        dirLinearNull[2]*self.MAXSPEEDLINEAR))
        
        velocityNull = np.concatenate((linearVelocityNull, np.zeros(3)), axis=None)

        return velocityEE, velocityNull

    def publishDataToROS(self):

        self.updateError()

        # determine velocities
        errorEE, errorNull         = self.getErrorVelocity()
        avoidanceEE, avoidanceNull = self.getPotentialFieldVelocity()

        # choose either restoring or avoidance velocity for null space
        if np.linalg.norm(avoidanceNull) != 0.0:
            errorNull = np.zeros(6)
        
        velocityEE   = (errorEE + avoidanceEE)* \
                        np.concatenate((np.ones(3)*self._modVelocityLinear, np.ones(3)))
        velocityNull = errorNull + avoidanceNull

        print(self._modVelocityLinear)
        print(np.round(velocityEE, 3))
        print(np.linalg.norm(velocityEE))
        print(np.round(velocityNull, 3))
        print(np.linalg.norm(velocityNull))
        print

        # update robot velocities and publish
        self.robotTwistCommand.twist.linear_x  = velocityEE[0]
        self.robotTwistCommand.twist.linear_y  = velocityEE[1]
        self.robotTwistCommand.twist.linear_z  = velocityEE[2]
        self.robotTwistCommand.twist.angular_x = velocityEE[3]
        self.robotTwistCommand.twist.angular_y = velocityEE[4]
        self.robotTwistCommand.twist.angular_z = velocityEE[5]

        self.robotNullTwistCommand.twist.linear_x  = velocityNull[0]
        self.robotNullTwistCommand.twist.linear_y  = velocityNull[1]
        self.robotNullTwistCommand.twist.linear_z  = velocityNull[2]
        self.robotNullTwistCommand.twist.angular_x = velocityNull[3]
        self.robotNullTwistCommand.twist.angular_y = velocityNull[4]
        self.robotNullTwistCommand.twist.angular_z = velocityNull[5]

        self.pubKinova.publish(self.robotTwistCommand)
        self.pubKinovaNull.publish(self.robotNullTwistCommand)
        # self.pubGripper.publish(self.gripperPose)


def main():

    rospy.init_node('path_planner')
    node = PathPlannerNode()

    print('Path Planner\n')

    while not rospy.is_shutdown():

        node.publishDataToROS()
        node.rate.sleep()


if __name__=="__main__":
	main()