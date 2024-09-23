#!/usr/bin/env python

# Gripper driver with position control and with filtering of only the average glove finger position data.

import roslib
import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

import actionlib
from control_msgs.msg import GripperCommandActionGoal
from kortex_driver.srv import *
from kortex_driver.msg import *

class GripperDriver():
    def __init__(self):
        self.current_position = 0
        self.desired_position = 0
        self.upperBound = 0
        self.lowerBound = 0
        self.noiseMargin = 10

        rospy.init_node('gripper_driver')

        self.send_gripper_command_full_name = '/my_gen3/base/send_gripper_command'
        rospy.wait_for_service(self.send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(self.send_gripper_command_full_name, SendGripperCommand)

        rospy.Subscriber("fingersPos", Float32MultiArray, self.fingersPosCallback)
        # rospy.Subscriber("/my_gen3/base_feedback", BaseCyclic_Feedback, self.baseFeedbackCB)

    # def baseFeedbackCB(self, msg):
        # self.current_position = msg.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position
        # print('Current gripper position: ' + str(self.current_position))

    def map(self, num, imuRange, jointRange):
        mapped = (((num - imuRange[0])/(imuRange[1]-imuRange[0]))*(jointRange[1]-jointRange[0])) + float((jointRange[0]))

        if mapped >= jointRange[1]:
            return jointRange[1]
        elif mapped <= jointRange[0]:
            return jointRange[0]
        else:
            return mapped

    def callMap(self, fingersPos):
        min_glove_readings = [190, 190, 190] #fully open
        max_glove_readings = [200, 210, 205] #fully closed

        min_robot = 0
        max_robot = 100

        final_readings = [0, 0, 0]

        for i in range(0, len(final_readings)):
            final_readings[i] = self.map(fingersPos[i], [min_glove_readings[i], max_glove_readings[i]], [min_robot, max_robot])

        return final_readings

    def fingersPosCallback(self, msg):
        mapped_vals = self.callMap(np.array(msg.data))
        final_pos = np.average(mapped_vals)
        self.desired_position = final_pos

    def send_gripper_command_func(self, val, OpMode):
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = val
        req.input.gripper.finger.append(finger)

        if (OpMode == 'vel'):
            req.input.mode = GripperMode.GRIPPER_SPEED
        elif (OpMode == 'pos'):
            req.input.mode = GripperMode.GRIPPER_POSITION
        else:
            req.input.mode = GripperMode.GRIPPER_POSITION

        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            # rospy.logerr("Failed to call SendGripperCommand")
            # print("Waiting for service...")
            # rospy.wait_for_service(self.send_gripper_command_full_name)
            return False
        else:
            # print("Gripper service data pushed.")
            return True

    def calculate_position(self): # in between 0 to 1
        if self.desired_position > self.upperBound or self.desired_position < self.lowerBound:
            self.upperBound = self.desired_position + self.noiseMargin
            if self.upperBound > 100:
                self.upperBound = 100
            self.lowerBound = self.desired_position - self.noiseMargin
            if self.lowerBound < 0:
                self.lowerBound = 0
        #print('Desired_Pos:' + str(self.desired_position))
        #print('Upperbound: ' + str(self.upperBound))
        #print('Lowerbound: ' + str(self.lowerBound))
        pos = self.upperBound / 100

        print('Gripper finger position [0: open, 1: close]: ' + str(pos))
        self.send_gripper_command_func(pos, 'pos')

if __name__ == '__main__':
    gd = GripperDriver()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        gd.calculate_position()
        rate.sleep()

    gd.send_gripper_command_func(0, 'pos')  # Fully open the gripper before closing this node.
