#!/usr/bin/env python

import rospy
import copy
import numpy as np

from scipy.spatial.transform import Rotation as r

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from mocap_interface.msg import MocapPose

from mocap_interface.srv import PropSpawn


class CommandRobot():

    def __init__(self, path):

        # list of points
        self.path = path
        self._pCounter = 0

        self._point = MocapPose()
        self._robotState = Pose()

        self._linearThreshold = 0.005
        self._angularThreshold = 0.5

        # ros spin rate
        self.rate = rospy.Rate(100)

        # publisher
        self._pubMocapPos = rospy.Publisher('/mocap_ee_pose', MocapPose, queue_size=10)

        # subscriber
        rospy.Subscriber('/j2n6s300_driver/out/tool_pose', PoseStamped, self._callbackKinovaPose)
    
    # check if robot is at desired destination point
    def _isRobotAtDestination(self):

        # position error
        xDiff = abs(self._point.pose.position.x - self._robotState.position.x)
        yDiff = abs(self._point.pose.position.y - self._robotState.position.y)
        zDiff = abs(self._point.pose.position.z - self._robotState.position.z)

        # angles
        try:
            anglesDest = r.from_quat((self._point.pose.orientation.x, self._point.pose.orientation.y, self._point.pose.orientation.z, \
                                        self._point.pose.orientation.w)).as_euler('xyz', degrees=True)
        except:
            anglesDest = r.from_quat((0, 0, 0, 1)).as_euler('xyz', degrees=True)

        try:
            anglesRobot = r.from_quat((self._robotState.pose.orientation.x, self._robotState.pose.orientation.y, self._robotState.pose.orientation.z, \
                                        self._robotState.pose.orientation.w)).as_euler('xyz', degrees=True)
        except:
            anglesRobot = r.from_quat((0, 0, 0, 1)).as_euler('xyz', degrees=True)
        
        # angles error
        xADiff = abs(anglesRobot[0] - anglesDest[0])
        yADiff = abs(anglesRobot[1] - anglesDest[1])
        zADiff = abs(anglesRobot[2] - anglesDest[2])

        # account for rotation around 0 and 360
        if xADiff >= 180.0:
            xADiff -= 360
        if yADiff >= 180.0:
            yADiff -= 360
        if zADiff >= 180.0:
            zADiff -= 360

        # determine if pose is reached
        if xDiff < self._linearThreshold and yDiff < self._linearThreshold and zDiff < self._linearThreshold and \
            xADiff < self._angularThreshold and yADiff < self._angularThreshold and zADiff < self._angularThreshold:
            return True
        else:
            return False

    # get robot ee location
    def _callbackKinovaPose(self, dataPoseStamped):

        self._robotState = dataPoseStamped.pose

    # publish commands
    def update(self):

        # update point with next destination
        self._point.pose.position.x = self.path[self._pCounter][0]
        self._point.pose.position.y = self.path[self._pCounter][1]
        self._point.pose.position.z = self.path[self._pCounter][2]

        self._point.pose.orientation.x = self.path[self._pCounter][3]
        self._point.pose.orientation.y = self.path[self._pCounter][4]
        self._point.pose.orientation.z = self.path[self._pCounter][5]
        self._point.pose.orientation.w = self.path[self._pCounter][6]

        self._point.grasp = self.path[self._pCounter][7]

        # wait for robot to reach destination
        if self._isRobotAtDestination():
            if self._pCounter < len(self.path)-1:
                self._pCounter += 1
            
            else:
                print('Path Completed\n')
                rospy.signal_shutdown('Path completed')

        self._pubMocapPos.publish(self._point)


def pick(pose):

    # initialize poses
    pose[7]  = False
    prevPick = copy.deepcopy(pose)
    pick     = copy.deepcopy(pose)
    postPick = copy.deepcopy(pose)

    # hover over picking pose
    prevPick[2] += 0.1

    # move and grasp at picking pose
    pick[7] = True

    # hover over picking pose while grasping
    postPick[2] += 0.1
    postPick[7] = True

    # return full sequence
    return [prevPick, pose, pick, postPick]

def place(pose):

    # initialize poses
    pose[7]  = True
    prevPlace = copy.deepcopy(pose)
    place     = copy.deepcopy(pose)
    postPlace = copy.deepcopy(pose)

    # hover over placing pose
    prevPlace[2] += 0.1
    prevPlace[7] = True

    # move and release grasp at placing pose
    place[7] = False

    # hover over placing pose while grasping
    postPlace[2] += 0.1
    postPlace[7] = False

    # return full sequence
    return [prevPlace, pose, place, postPlace]

def main():

    # set up service calls
    spawnObject = rospy.ServiceProxy('/prop_spawner', PropSpawn)

    # stack up bricks
    pose = Pose()
    pose.position.x = -0.35
    pose.position.y = -0.35
    pose.position.z = 0.015
    spawnObject('brick', pose, 'g')
    pose.position.z += 0.03
    spawnObject('brick', pose, 'g')
    pose.position.z += 0.03
    spawnObject('brick', pose, 'g')
    pose.position.z += 0.03
    spawnObject('brick', pose, 'g')
    pose.position.y = -0.20
    pose.position.z = 0.015
    spawnObject('brick', pose, 'g')
    pose.position.z += 0.03
    spawnObject('brick', pose, 'g')
    pose.position.z += 0.03
    spawnObject('brick', pose, 'g')
    pose.position.z += 0.03
    spawnObject('brick', pose, 'g')
    pose.position.z += 0.03
    spawnObject('brick', pose, 'g')

    # points to spell out 'WPI'
    wpi = [(-0.36, -0.30, 0.35, 1, 0, 0, 0, False),
           (-0.36, -0.36, 0.35, 1, 0, 0, 0, False),
           (-0.30, -0.36, 0.15, 1, 0, 0, 0, False),
           (-0.24, -0.36, 0.25, 1, 0, 0, 0, False),
           (-0.18, -0.36, 0.15, 1, 0, 0, 0, False),
           (-0.12, -0.36, 0.35, 1, 0, 0, 0, False),
           (-0.12, -0.30, 0.35, 1, 0, 0, 0, False),
           (-0.06, -0.30, 0.15, 1, 0, 0, 0, False),
           (-0.06, -0.36, 0.15, 1, 0, 0, 0, False),
           (-0.06, -0.36, 0.35, 1, 0, 0, 0, False),
           ( 0.12, -0.36, 0.35, 1, 0, 0, 0, False),
           ( 0.18, -0.36, 0.30, 1, 0, 0, 0, False),
           ( 0.12, -0.36, 0.25, 1, 0, 0, 0, False),
           (-0.06, -0.36, 0.25, 1, 0, 0, 0, False),
           (-0.06, -0.30, 0.25, 1, 0, 0, 0, False),
           ( 0.24, -0.30, 0.15, 1, 0, 0, 0, False),
           ( 0.24, -0.36, 0.15, 1, 0, 0, 0, False),
           ( 0.36, -0.36, 0.15, 1, 0, 0, 0, False),
           ( 0.36, -0.30, 0.15, 1, 0, 0, 0, False),
           ( 0.24, -0.30, 0.35, 1, 0, 0, 0, False),
           ( 0.24, -0.36, 0.35, 1, 0, 0, 0, False),
           ( 0.36, -0.36, 0.35, 1, 0, 0, 0, False),
           ( 0.30, -0.36, 0.35, 1, 0, 0, 0, False),
           ( 0.30, -0.36, 0.15, 1, 0, 0, 0, False),
           ( 0.30, -0.30, 0.15, 1, 0, 0, 0, False),
           ( 0.00, -0.25, 0.25, 1, 0, 0, 0, False)]
    
    # pyramid blueprint
    pyramidBP = [[0.35, -0.45, 0.015, 1, 0, 0, 0, False], \
                [0.35, -0.35, 0.015, 1, 0, 0, 0, False], \
                [0.35, -0.25, 0.015, 1, 0, 0, 0, False], \
                [0.35, -0.15, 0.015, 1, 0, 0, 0, False], \
                [0.35, -0.40, 0.045, 1, 0, 0, 0, False], \
                [0.35, -0.30, 0.045, 1, 0, 0, 0, False], \
                [0.35, -0.20, 0.045, 1, 0, 0, 0, False], \
                [0.35, -0.35, 0.075, 1, 0, 0, 0, False], \
                [0.35, -0.25, 0.075, 1, 0, 0, 0, False]]

    # pyramid stacking
    pyramid = [(-0.15, -0.35, 0.35, 1, 0, 0, 0, False)]
    pyramid += pick([-0.35, -0.20, 0.135, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.45, 0.015, 1, 0, 0, 0, False])
    pyramid += pick([-0.35, -0.20, 0.105, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.35, 0.015, 1, 0, 0, 0, False])
    pyramid += pick([-0.35, -0.20, 0.075, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.25, 0.015, 1, 0, 0, 0, False])
    pyramid += pick([-0.35, -0.35, 0.105, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.15, 0.015, 1, 0, 0, 0, False])
    pyramid += pick([-0.35, -0.35, 0.075, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.40, 0.045, 1, 0, 0, 0, False])
    pyramid += pick([-0.35, -0.20, 0.045, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.30, 0.045, 1, 0, 0, 0, False])
    pyramid += pick([-0.35, -0.20, 0.015, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.20, 0.045, 1, 0, 0, 0, False])
    pyramid += pick([-0.35, -0.35, 0.045, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.35, 0.075, 1, 0, 0, 0, False])
    pyramid += pick([-0.35, -0.35, 0.015, 1, 0, 0, 0, False])
    pyramid += place([0.35, -0.25, 0.075, 1, 0, 0, 0, False])
    pyramid.append((-0.15, -0.35, 0.35, 1, 0, 0, 0, False))

    # brick grid test
    grid = [(-0.15, -0.35, 0.35, 1, 0, 0, 0, False)]
    grid += pick([-0.35, -0.20, 0.135, 1, 0, 0, 0, False])
    grid += place([0.35, -0.45, 0.015, 1, 0, 0, 0, False])
    grid += pick([-0.35, -0.20, 0.105, 1, 0, 0, 0, False])
    grid += place([0.35, -0.35, 0.015, 1, 0, 0, 0, False])
    grid += pick([-0.35, -0.20, 0.075, 1, 0, 0, 0, False])
    grid += place([0.35, -0.25, 0.015, 1, 0, 0, 0, False])
    grid += pick([-0.35, -0.35, 0.105, 1, 0, 0, 0, False])
    grid += place([0.35, -0.15, 0.015, 1, 0, 0, 0, False])
    grid += pick([-0.35, -0.35, 0.075, 1, 0, 0, 0, False])
    grid += place([0.40, -0.40, 0.045, 1, 0, 0, 0, False])
    grid += pick([-0.35, -0.20, 0.045, 1, 0, 0, 0, False])
    grid += place([0.40, -0.30, 0.045, 1, 0, 0, 0, False])
    grid += pick([-0.35, -0.20, 0.015, 1, 0, 0, 0, False])
    grid += place([0.40, -0.20, 0.045, 1, 0, 0, 0, False])
    grid += pick([-0.35, -0.35, 0.045, 1, 0, 0, 0, False])
    grid += place([0.30, -0.35, 0.045, 1, 0, 0, 0, False])
    grid += pick([-0.35, -0.35, 0.015, 1, 0, 0, 0, False])
    grid += place([0.30, -0.25, 0.045, 1, 0, 0, 0, False])
    grid.append((-0.15, -0.35, 0.35, 1, 0, 0, 0, False))

    rospy.init_node('robot_control')
    # node = CommandRobot(wpi)
    node = CommandRobot(pyramid)
    # node = CommandRobot(grid)

    print('Start sequence')

    while not rospy.is_shutdown():

        node.update()
        node.rate.sleep()


if __name__=="__main__":
	main()