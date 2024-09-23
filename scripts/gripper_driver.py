#!/usr/bin/env python

import roslib
import rospy

import sys
import numpy as np
import random
import itertools
import csv
import os
import time

import serial

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


        rospy.init_node('gripper_driver')

        self.send_gripper_command_full_name = '/my_gen3/base/send_gripper_command'
        rospy.wait_for_service(self.send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(self.send_gripper_command_full_name, SendGripperCommand)

        rospy.Subscriber("fingersPos", Float32MultiArray, self.fingersPosCallback)
        rospy.Subscriber("/my_gen3/base_feedback", BaseCyclic_Feedback, self.baseFeedbackCB)

    def baseFeedbackCB(self, msg):
        self.current_position = msg.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position
        #print(self.current_position)


    def map(self, num, imuRange, jointRange):
        mapped = (((num - imuRange[0])/(imuRange[1]-imuRange[0]))*(jointRange[1]-jointRange[0])) + float((jointRange[0]))

        if mapped >= jointRange[1]:
            return jointRange[1]
        elif mapped <= jointRange[0]:
            return jointRange[0]
        else:
            return mapped

    def callMap(self, fingersPos):
        #NO HAPTICS
        min_glove_readings = [198, 199, 194] #fully open
        max_glove_readings = [214, 212, 202] #fully closed

        #FABRIC ONES
        # min_glove_readings = [188, 196, 211]
        # max_glove_readings = [244, 235, 234]

        min_robot = 0
        max_robot = 100

        final_readings = [0, 0, 0]

        for i in range(0, len(final_readings)):
            final_readings[i] = self.map(fingersPos[i], [min_glove_readings[i], max_glove_readings[i]], [min_robot, max_robot])

        return final_readings


    def fingersPosCallback(self, msg):
        mapped_vals = self.callMap(np.array(msg.data))
        #print(mapped_vals)
        mapped_vals = [mapped_vals[1], mapped_vals[2]]

        final_pos = np.average(mapped_vals)
        self.desired_position = final_pos


    def send_gripper_command_func(self, val, mode):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        #print('val = ', val)

        finger.value = val

        req.input.gripper.finger.append(finger)

        if (mode == 'vel'):
            req.input.mode = GripperMode.GRIPPER_SPEED
        if (mode == 'pos'):
            req.input.mode = GripperMode.GRIPPER_POSITION


        #rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            #rospy.logerr("Failed to call SendGripperCommand")

            print("Failed call")
            # print("Waiting for service...")
            # rospy.wait_for_service(self.send_gripper_command_full_name)

            return False
        else:
            #time.sleep(0.05)
            return True

    def calculate_velocity(self):
        #self.desired_position = 10
        err = self.current_position - self.desired_position

        if abs(err) < 2.0:
            pd = 0.001
        else:
            pd = 0.008

        vel = pd * err

        # if abs(vel)<0.08:
        # 	vel = 0

        if abs(vel) > 1:
            vel = 1 * np.sign(vel)

        msg = GripperCommandActionGoal()
        msg.goal.command.position = vel

        #print(err)


        self.send_gripper_command_func(vel, 'vel')

        # print(round(err,2), round(self.current_position, 2), round(self.desired_position,2), round(vel,3))

if __name__ == '__main__':
    gd = GripperDriver()
    rate = rospy.Rate(100)

    #gd.send_gripper_command_func(0, 'pos')
    #time.sleep(1)

    while not rospy.is_shutdown():
        gd.calculate_velocity()
        rate.sleep()

    gd.send_gripper_command_func(0, 'vel')
