#!/usr/bin/env python

import rospy
import rospkg
import time
import numpy as np

from mocap_interface.srv import PropSpawn,  PropSpawnResponse
from mocap_interface.srv import PropDelete, PropDeleteResponse
from mocap_interface.srv import PropClear,  PropClearResponse

from mocap_interface.msg import MocapPose
from mocap_interface.msg import Prop
from mocap_interface.msg import Props
from geometry_msgs.msg   import Pose
from geometry_msgs.msg   import PoseStamped
from geometry_msgs.msg   import PoseWithCovarianceStamped
from geometry_msgs.msg   import TransformStamped
from gazebo_msgs.msg     import ModelState
from gazebo_msgs.msg     import ModelStates
from kortex_driver.msg   import BaseFeedback
from kortex_driver.msg   import BaseCyclic_Feedback
#from apriltag_ros.msg    import AprilTagDetection
#from apriltag_ros.msg    import AprilTagDetectionArray
from tf.msg              import tfMessage

from scipy.spatial.transform import Rotation as r

from gazebo_msgs.srv import SpawnModel, DeleteModel

# disable scientific notation
np.set_printoptions(suppress=True)

class PropManager:

    def __init__(self):

        # get package path
        self._rospack = rospkg.RosPack()
        self._pkgPath = self._rospack.get_path('mocap_interface')

        # open model files
        with open(self._pkgPath + '/gazebo/pointmarker/model.sdf') as f:
            self.modelPoint = f.read()
        with open(self._pkgPath + '/gazebo/obstacle/model.sdf') as f:
            self.modelObstacle = f.read()
        with open(self._pkgPath + '/gazebo/marker/model.sdf') as f:
            self.modelMarker = f.read()
        with open(self._pkgPath + '/gazebo/frame/model.sdf') as f:
            self.modelFrame = f.read()
        with open(self._pkgPath + '/gazebo/brick/model.sdf') as f:
            self.modelBrick = f.read()
        with open(self._pkgPath + '/gazebo/bno055/model.sdf') as f:
            self.modelBNO = f.read()
        with open(self._pkgPath + '/gazebo/mocap_ee/model.sdf') as f:
            self.modelMocapEE = f.read()
        with open(self._pkgPath + '/gazebo/block_2in/model.sdf') as f:
            self.modelBlock2in = f.read()
        with open(self._pkgPath + '/gazebo/bin/model.sdf') as f:
            self.modelBin = f.read()
        with open(self._pkgPath + '/gazebo/box1/model.sdf') as f:
            self.modelBox1 = f.read()
        with open(self._pkgPath + '/gazebo/post/model.sdf') as f:
            self.modelPost = f.read()

        # keep track of end effector pose
        self._eePosition    = np.zeros(3)
        self._eeOrientation = np.zeros(4)
        self._eeFingers     = np.zeros(3)

        # keep track of objects
        self._props = Props()
        self._robot = Pose()
        self._robot.orientation.w = 1.0

        # grasping
        self._isGrasping = False
        self._graspThreshold = 0.03 # m
        self._graspedObject = ''
        
        # ros spin rate
        self.rate = rospy.Rate(100)
        
        # robot type
        self._robotName = 'my_gen3'

        # services
        self._serviceSpawn  = rospy.Service('prop_spawner', PropSpawn,  self._handlePropSpawn)
        self._serviceDelete = rospy.Service('prop_deleter', PropDelete, self._handlePropDelete)
        self._serviceClear  = rospy.Service('prop_clearer', PropClear,  self._handlePropClear)

        # publishers
        self._pubGazebo = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self._pubProps  = rospy.Publisher('/props',                  Props,      queue_size=10)
        
        # subscribers
        # rospy.Subscriber('/gazebo/model_states',     ModelStates, self._callbackGazebo)
        rospy.Subscriber('/mocap_ee_pose',           MocapPose,   self._callbackMocapPose)
        rospy.Subscriber('/' + self._robotName + '/base_feedback', BaseCyclic_Feedback, self._callbackBaseFeedback)
        rospy.Subscriber('/' + self._robotName + '', PoseStamped, self._callbackCommandGripper)
        rospy.Subscriber('/tf',                      tfMessage, self._callbackTF)

        # setup gazebo
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        rospy.wait_for_service('gazebo/delete_model')

        self._spawn  = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        self._delete = rospy.ServiceProxy('gazebo/delete_model',    DeleteModel)

        # spawn in destination frame
        self._spawn('mocap_frame', self.modelFrame,   '', Pose(), 'world')
        self._spawn('mocap_box',   self.modelMocapEE, '', Pose(), 'world')

        # spawn in robot frame
        self._spawn('robot_frame', self.modelFrame, '', Pose(), 'world')

        # set shutdown behavior
        rospy.on_shutdown(self._onShutdown)

    # shutdown process
    def _onShutdown(self):

        self.clearProps()

    def _handlePropSpawn(self, msg):

        success = self._propSpawn(msg.object, msg.name, msg.type, msg.pose)

        return PropSpawnResponse(success)
    
    # spawn a single sdf or urdf
    def _propSpawn(self, propObject, propName, propType, propPose):

        # verify object model
        if propObject == 'point':
            model = self.modelPoint
        elif propObject == 'obstacle':
            model = self.modelObstacle
        elif propObject == 'marker':
            model = self.modelMarker
        elif propObject == 'frame':
            model = self.modelFrame
        elif propObject == 'brick':
            model = self.modelBrick
        elif propObject == 'block2in':
            model = self.modelBlock2in
        elif propObject == 'bin':
            model = self.modelBin
        elif propObject == 'box1':
            model = self.modelBox1
        elif propObject == 'post':
            model = self.modelPost
        else:
            return PropSpawnResponse(False)

        # build name
        if propName == '':
            num  = len(self._props.props) + 1
            name = propType + '_' + propObject + '_' + str(num)
        else:
            name = propName

        # verify object type
        # obstacle
        # graspable
        # prop (intangible)
        if propType == 'o':
            grasp = False

        elif propType == 'g':
            grasp = True

        elif propType == 'p':
            grasp = False
            
        else:
            return PropSpawnResponse(False)
        
        # build prop and add to manager list
        prop           = Prop()
        prop.name      = name
        prop.pose      = propPose
        prop.graspable = grasp

        self._props.props.append(prop)

        # spawn in gazebo
        self._spawn(name, model, '', prop.pose, 'world')
        return PropSpawnResponse(True)
    
    def _callbackMocapPose(self, dataMocapPose):

        # get mocap pose and transfer it to gazebo
        frame = ModelState()
        frame.model_name = 'mocap_frame'
        frame.pose = dataMocapPose.pose

        box = ModelState()
        box.model_name = 'mocap_box'
        box.pose = dataMocapPose.pose

        # publish poses to gazebo
        self._pubGazebo.publish(frame)
        self._pubGazebo.publish(box)
    
    # delete an object from gazebo
    def _handlePropDelete(self, msg):

        for prop in self._props.props:
            if prop.name == msg.name:

                # remove prop from manager list and delete from gazebo
                self._props.props.remove(prop)
                self._delete(msg.name)

                return PropDeleteResponse(True)
            
            else:
                return PropDeleteResponse(False)
        
        return PropDeleteResponse(False)
    
    # clear all objects from gazebo
    def _handlePropClear(self, msg):

        self.clearProps()

        # verify all objects were cleared
        if len(self._props.props) == 0:
            return PropClearResponse(True)
        
        else:
            return PropClearResponse(False)
    
    # get end-effector pose
    def _callbackBaseFeedback(self, dataFeedback):

        # update current pose
        self._eePosition[0] = dataFeedback.base.tool_pose_x
        self._eePosition[1] = dataFeedback.base.tool_pose_y
        self._eePosition[2] = dataFeedback.base.tool_pose_z

        # convert orientation to quaternion
        angles = np.zeros(3)
        angles[0] = dataFeedback.base.tool_pose_theta_x
        angles[1] = dataFeedback.base.tool_pose_theta_y
        angles[2] = dataFeedback.base.tool_pose_theta_z

        self._eeOrientation = r.from_euler('xyz', angles, degrees=True).as_quat()
    
    # get gripper pose
    def _callbackCommandGripper(self, dataFingerPosition):

        self._eeFingers[0] = dataFingerPosition.finger1
        self._eeFingers[1] = dataFingerPosition.finger2
        self._eeFingers[2] = dataFingerPosition.finger3

        if self._eeFingers[0] > 0.5:
            self._isGrasping = True

        else:
            self._isGrasping = False
    
    # get apriltag transforms
    def _callbackTF(self, dataTFMessage):

        # update apriltag positions or spawn in new objects as needed
        for det in dataTFMessage.transforms:

            isNewProp = True

            # check for robot pose
            if det.child_frame_id == 'robot':

                # build robot transform
                rRobot = r.from_quat((det.transform.rotation.x, det.transform.rotation.y,
                                        det.transform.rotation.z, det.transform.rotation.w)).as_dcm()
                pRobot = np.reshape(np.array((det.transform.translation.x, det.transform.translation.y,
                                        det.transform.translation.z)), (3, 1))
                tRobot = np.vstack((np.hstack((rRobot, pRobot)), np.array((0, 0, 0, 1))))

                # convert to correct frame
                pRobotNew = tRobot[:3, 3].T
                rRobotNew = r.from_dcm(tRobot[:3, :3]).as_quat()

                self._robot.position.x    = pRobotNew[0]
                self._robot.position.y    = pRobotNew[1]
                self._robot.position.z    = pRobotNew[2]
                self._robot.orientation.x = rRobotNew[0]
                self._robot.orientation.y = rRobotNew[1]
                self._robot.orientation.z = rRobotNew[2]
                self._robot.orientation.w = rRobotNew[3]
            
            else:

                # find prop wrt robot
                rRobot = r.from_quat((self._robot.orientation.x, self._robot.orientation.y,
                                        self._robot.orientation.z, self._robot.orientation.w)).as_dcm()
                pRobot = np.reshape(np.array((self._robot.position.x, self._robot.position.y,
                                        self._robot.position.z)), (3, 1))
                tRobot = np.vstack((np.hstack((rRobot, pRobot)), np.array((0, 0, 0, 1))))
                
                rProp = r.from_quat((det.transform.rotation.x, det.transform.rotation.y,
                                        det.transform.rotation.z, det.transform.rotation.w)).as_dcm()
                pProp = np.reshape(np.array((det.transform.translation.x, det.transform.translation.y,
                                        det.transform.translation.z)), (3, 1))
                tProp = np.vstack((np.hstack((rProp, pProp)), np.array((0, 0, 0, 1))))

                tRobotToProp = np.matmul(np.linalg.inv(tRobot), tProp)
                pRobotToProp = tRobotToProp[:3, 3].T
                rRobotToProp = r.from_dcm(tRobotToProp[:3, :3]).as_quat()
                
                # get detection pose
                newPose = Pose()
                newPose.position.x    = pRobotToProp[0]
                newPose.position.y    = pRobotToProp[1]
                newPose.position.z    = pRobotToProp[2]
                newPose.orientation.x = rRobotToProp[0]
                newPose.orientation.y = rRobotToProp[1]
                newPose.orientation.z = rRobotToProp[2]
                newPose.orientation.w = rRobotToProp[3]
            
                for prop in self._props.props:

                    # update pose if prop already exists
                    if det.child_frame_id == prop.name:
                        prop.pose = newPose
                        isNewProp = False

                # spawn new prop if it doesn't exist already
                if isNewProp:
                    tok = det.child_frame_id.split('_')
                    self._propSpawn(tok[1], det.child_frame_id, tok[0], newPose)
        
    def clearProps(self):

        # delete all objects
        for prop in self._props.props:
            self._delete(prop.name)
            time.sleep(0.1)

        # clear lists
        self._props.props = []
    
    def publishDataToROS(self):

        # stick grasped object to gripper
        if self._isGrasping and self._graspedObject != '':

            grasped = ModelState()
            grasped.model_name = self._graspedObject
            grasped.pose.position.x = self._eePosition[0]
            grasped.pose.position.y = self._eePosition[1]
            grasped.pose.position.z = self._eePosition[2]
            grasped.pose.orientation.x = self._eeOrientation[0]
            grasped.pose.orientation.y = self._eeOrientation[1]
            grasped.pose.orientation.z = self._eeOrientation[2]
            grasped.pose.orientation.w = self._eeOrientation[3]

            self._pubGazebo.publish(grasped)
        
        # show robot ee frame
        eeFrame = ModelState()
        eeFrame.model_name = 'robot_frame'
        eeFrame.pose.position.x = self._eePosition[0]
        eeFrame.pose.position.y = self._eePosition[1]
        eeFrame.pose.position.z = self._eePosition[2]
        eeFrame.pose.orientation.x = self._eeOrientation[0]
        eeFrame.pose.orientation.y = self._eeOrientation[1]
        eeFrame.pose.orientation.z = self._eeOrientation[2]
        eeFrame.pose.orientation.w = self._eeOrientation[3]

        self._pubGazebo.publish(eeFrame)

        # update the rest of the props in gazebo
        for prop in self._props.props:

            propState = ModelState()
            propState.model_name = prop.name
            propState.pose = prop.pose

            self._pubGazebo.publish(propState)
        
        # publish prop data
        self._pubProps.publish(self._props)


def main():

    rospy.init_node('prop_manager')
    node = PropManager()

    print('Prop Manager\n')

    while not rospy.is_shutdown():
        
        node.publishDataToROS()
        node.rate.sleep()


if __name__ == "__main__":
    main()
