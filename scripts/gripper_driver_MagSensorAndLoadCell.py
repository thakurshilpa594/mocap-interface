#!/usr/bin/env python

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
        self.current_position = 500
        self.upperBound = 680
        self.lowerBound = 500
        self.change = 0

        rospy.init_node('gripper_driver')

        self.send_gripper_command_full_name = '/my_gen3/base/send_gripper_command'
        rospy.wait_for_service(self.send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(self.send_gripper_command_full_name, SendGripperCommand)

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
            #rospy.logerr("Failed to call SendGripperCommand")
            print("Gripper service call failed!")
            # print("Waiting for service...")
            # rospy.wait_for_service(self.send_gripper_command_full_name)
            return False
        else:
            print("Gripper service data pushed.")
            return True

    def calculate_position(self): # in between 0 to 1
        if self.current_position >= self.upperBound:
            self.change = -1
        if self.current_position <= self.lowerBound:
            self.change = 1
        
        self.current_position = self.current_position + self.change

        pos = self.current_position / 1000
        print('Gripper finger position [0: open, 1: close]: ' + str(pos))
        self.send_gripper_command_func(pos, 'pos')

if __name__ == '__main__':
    gd = GripperDriver()
    rate = rospy.Rate(100)

    gd.send_gripper_command_func(0, 'pos')

    while not rospy.is_shutdown():
        gd.calculate_position()
        rate.sleep()

    gd.send_gripper_command_func(0, 'pos')
