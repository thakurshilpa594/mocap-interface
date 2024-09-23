#!/usr/bin/env python

import rospy
import numpy as np
import sys
import time
import matplotlib.pyplot as plt
import datetime
from std_msgs.msg import Int16

from sensor_msgs.msg     import JointState
from mocap_interface.msg import MocapPose
from kortex_driver.msg   import BaseCyclic_Feedback

from scipy.spatial.transform import Rotation as r

# disable scientific notation
np.set_printoptions(suppress=True)

class Logdata:

    def __init__(self, directory, fileName):

        self._robotName = 'my_gen3'
        self._startTime   = time.time()

        # data to collect
        # self._robotJoints = np.zeros(7)
        # self._robotPose   = np.zeros(6)
        # self._mocapPose   = np.zeros(6)
        self.gripperSensorReading = 0
        self.hapticFeedbackValue = 0
        self.samples = 200
        self.NumOfxDataPoints = 30
        self.x_datapoints = np.linspace(0, self.NumOfxDataPoints, self.samples+1)
        self.y1_datapoints = np.zeros(len(self.x_datapoints))
        self.y2_datapoints = np.zeros(len(self.x_datapoints))

        # build up name
        timeNow   = datetime.datetime.now()
        timestamp = '_' + str(timeNow.year) + '-' + str(timeNow.month).zfill(2) + '-' + str(timeNow.day).zfill(2) + \
                    '_' + str(timeNow.hour).zfill(2) + ':' + str(timeNow.minute).zfill(2)

        # csv file
        self._fileCSV  = open(directory + '/' + fileName + timestamp + '.csv', 'w')

        # ros spin rate
        self.rate = rospy.Rate(100)

        # subscribers
        # rospy.Subscriber('/mocap_ee_pose',                         MocapPose,           self._callbackMocapPose)
        # rospy.Subscriber('/' + self._robotName + '/base_feedback', BaseCyclic_Feedback, self._callbackBaseFeedback)
        # rospy.Subscriber('/' + self._robotName + '/joint_states',  JointState,          self._callbackJointAngles)
        rospy.Subscriber("/gripper_sensors", Int16, self.forceSensorCBF)
        rospy.Subscriber("/haptic_all", Int16, self.hapticDataCBF)

        # set shutdown behavior
        rospy.on_shutdown(self._onShutdown)

    def update_ydata(self, y1_new, y2_new):
        for i in range(len(self.x_datapoints)-1):
            self.y1_datapoints[i] = self.y1_datapoints[i+1]
            self.y2_datapoints[i] = self.y2_datapoints[i+1]
        self.y1_datapoints[-1] = y1_new
        self.y2_datapoints[-1] = y2_new

    def _onShutdown(self):
        currentTime = time.time() - self._startTime
        self._fileCSV.close()

        print("---------------------------------")
        print('File closed...')
        print('Run time: ' + str(np.round(currentTime, 2)) + 's')
        print("---------------------------------")

    # get mocap end-effector position
    # def _callbackMocapPose(self, dataMocapPose):
    #     try:
    #         angles = r.from_quat((dataMocapPose.pose.orientation.x, dataMocapPose.pose.orientation.y, dataMocapPose.pose.orientation.z, \
    #                                 dataMocapPose.pose.orientation.w)).as_euler('xyz', degrees=False)
    #     except:
    #         angles = r.from_quat((0, 0, 0, 1)).as_euler('xyz', degrees=False)

        # update mocap pose
        # self._mocapPose[0] = dataMocapPose.pose.position.x
        # self._mocapPose[1] = dataMocapPose.pose.position.y
        # self._mocapPose[2] = dataMocapPose.pose.position.z
        # self._mocapPose[3] = angles[0]
        # self._mocapPose[4] = angles[1]
        # self._mocapPose[5] = angles[2]

    # get robot end-effector pose
    # def _callbackBaseFeedback(self, dataFeedback):

    #     quat = r.from_euler('xyz', (np.deg2rad(dataFeedback.base.tool_pose_theta_x),
    #                         np.deg2rad(dataFeedback.base.tool_pose_theta_y),
    #                         np.deg2rad(dataFeedback.base.tool_pose_theta_z))).as_quat()

    #     # update current pose
    #     self._robotPose[0] = dataFeedback.base.tool_pose_x
    #     self._robotPose[1] = dataFeedback.base.tool_pose_y
    #     self._robotPose[2] = dataFeedback.base.tool_pose_z
    #     self._robotPose[3] = np.deg2rad(dataFeedback.base.tool_pose_theta_x)
    #     self._robotPose[4] = np.deg2rad(dataFeedback.base.tool_pose_theta_y)
    #     self._robotPose[5] = np.deg2rad(dataFeedback.base.tool_pose_theta_z)

    #     # modify tool pose to work with weird kinova representation
    #     self._robotPose[3:] = r.from_quat(quat).as_euler('xyz', degrees=False)

    # # update model joint angles
    # def _callbackJointAngles(self, dataJointState):

    #     # update joint angles
    #     self._robotJoints[0] = dataJointState.position[0]
    #     self._robotJoints[1] = dataJointState.position[1]
    #     self._robotJoints[2] = dataJointState.position[2]
    #     self._robotJoints[3] = dataJointState.position[3]
    #     self._robotJoints[4] = dataJointState.position[4]
    #     self._robotJoints[5] = dataJointState.position[5]
    #     self._robotJoints[6] = dataJointState.position[6]

    def forceSensorCBF(self, msg):
        rawForces = msg.data
        forceConstant = 5  # Load-cell calibrated value of 15 to convert magnetic sensor reading to grams.
        self.gripperSensorReading = forceConstant * rawForces

    def hapticDataCBF(self, msg):
        self.hapticFeedbackValue = msg.data

    # write data to file
    def recordData(self):

        currentTime = time.time() - self._startTime

        # mocapData = str(self._mocapPose[0]) + ',' + \
        #             str(self._mocapPose[1]) + ',' + \
        #             str(self._mocapPose[2]) + ',' + \
        #             str(self._mocapPose[3]) + ',' + \
        #             str(self._mocapPose[4]) + ',' + \
        #             str(self._mocapPose[5])

        # robotData = str(self._robotPose[0]) + ',' + \
        #             str(self._robotPose[1]) + ',' + \
        #             str(self._robotPose[2]) + ',' + \
        #             str(self._robotPose[3]) + ',' + \
        #             str(self._robotPose[4]) + ',' + \
        #             str(self._robotPose[5])

        # jointData = str(self._robotJoints[0]) + ',' + \
        #             str(self._robotJoints[1]) + ',' + \
        #             str(self._robotJoints[2]) + ',' + \
        #             str(self._robotJoints[3]) + ',' + \
        #             str(self._robotJoints[4]) + ',' + \
        #             str(self._robotJoints[5]) + ',' + \
        #             str(self._robotJoints[6])

        #self._fileCSV.write(str(currentTime) + ',' + str(mocapData) + ',' + str(robotData) + ',' + str(jointData) + '\n')
        self._fileCSV.write(str(currentTime) + ',' + str(self.gripperSensorReading) + ',' + str(self.hapticFeedbackValue) + '\n')
        self.update_ydata(self.gripperSensorReading,self.hapticFeedbackValue)


def main():
    fileName  = 'user1_Alec'
    directory = '/home/srl/Documents/UserStudyData'

    # get input arguments
    for i in range(len(sys.argv)):
        if sys.argv[i] == '-f' and len(sys.argv) >= i:
            fileName = sys.argv[i+1]
        elif sys.argv[i] == '-d' and len(sys.argv) >= i:
            directory = sys.argv[i+1]

    rospy.init_node('collect_data')
    node = Logdata(directory, fileName)

    plt.ion()
    fig = plt.figure()
    plt.grid(True)
    ax = fig.add_subplot(111)
    ax.set_title('Gripping Force and Haptic Feedback vs. Time', fontsize = 35)
    ax.set_xlabel('Time (s)', fontsize = 35)
    ax.set_ylabel('Force (g), Haptic Feedback (PWM)', fontsize = 35)
    plt.xticks(fontsize = 30)
    plt.yticks(fontsize = 30)
    line1, = ax.plot(node.x_datapoints, node.y1_datapoints, 'b-', linewidth = 3)
    line2, = ax.plot(node.x_datapoints, node.y2_datapoints, 'r-', linewidth = 3)
    ax.set_ybound([0, 500])
    ax.set_xbound([0, node.NumOfxDataPoints])
    ax.legend([line1, line2], ['Gripper Force Sensor', 'Haptic Feedback'], loc = 'upper left', fontsize = 30)

    print('Collecting and Plotting Experiment Data\n')

    while not rospy.is_shutdown():
        node.rate.sleep()
        node.recordData()
        line1.set_ydata(node.y1_datapoints)
        line2.set_ydata(node.y2_datapoints)
        fig.canvas.draw()
        fig.canvas.flush_events()


if __name__ == "__main__":
    main()
