#!/usr/bin/env python

# Gripper driver with position control and with filtering of each glove finger data.

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
        self.desiredGripperPosition = 0
        self.gripperFingerNoiseMargin = 6

        self.gloveFingerNoiseMargin = 4
        self.glove_readings_filter_buffer = [0, 0, 0, 0, 0]  # Array to store filtered glove finger readings.

        rospy.init_node('gripper_driver')

        self.send_gripper_command_full_name = '/my_gen3/base/send_gripper_command'
        rospy.wait_for_service(self.send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(self.send_gripper_command_full_name, SendGripperCommand)

        rospy.Subscriber("fingersPos", Float32MultiArray, self.fingersPosCallback)

    def map(self, num, numRange, jointRange):
        mapped = (((num - numRange[0])/(numRange[1]-numRange[0]))*(jointRange[1]-jointRange[0])) + float((jointRange[0]))

        if mapped >= jointRange[1]:
            return jointRange[1]
        elif mapped <= jointRange[0]:
            return jointRange[0]
        else:
            return mapped

    def filterAndMap(self, fingersPos):
        min_glove_readings = [189, 186, 184, 187, 192]  # Fully open finger readings.
        max_glove_readings = [217, 214, 204, 203, 203]  # Fully closed finger readings.

        minGripper = 0    # Gripper is fully open.
        maxGripper = 100  # Gripper is fully closed.
        mappedGripperPositions = [0, 0, 0, 0, 0]

        for i in range(0, len(mappedGripperPositions)):
            # Filter by tolerating a positive or negative deviation in the input by a magnitude of the noise margin.
            if fingersPos[i] > (self.glove_readings_filter_buffer[i] + self.gloveFingerNoiseMargin) or fingersPos[i] < (self.glove_readings_filter_buffer[i] - self.gloveFingerNoiseMargin):
                self.glove_readings_filter_buffer[i] = fingersPos[i]
            # Map the filtered glove finger readings to the gripper's actuation ranges.
            mappedGripperPositions[i] = self.map(self.glove_readings_filter_buffer[i], [min_glove_readings[i], max_glove_readings[i]], [minGripper, maxGripper])

        return mappedGripperPositions

    def fingersPosCallback(self, msg):
        inputGloveFingerData = np.array(msg.data)  # Convert to a numpy array.
        inputGloveFingerData = inputGloveFingerData.astype(int)  # Convert each element to an integer.
        mappedGripperFingerPositions = self.filterAndMap(inputGloveFingerData)  # Filter and map each glove finger data to gripper position.
        twoFingerGripperPosition = np.average(mappedGripperFingerPositions)  # Take the average of gripper finger positions to contoll a two finger gripper.
        twoFingerGripperPosition = int(twoFingerGripperPosition)  # Ignore the post decimal values after averaging.
        # Filter averaged value.
        if twoFingerGripperPosition > (self.desiredGripperPosition + self.gripperFingerNoiseMargin) or twoFingerGripperPosition < (self.desiredGripperPosition - self.gripperFingerNoiseMargin):
            self.desiredGripperPosition = twoFingerGripperPosition

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
            return False
        else:
            # print("Gripper service data pushed.")
            return True

    def setGripperPosition(self): # In between 0 to 1 as 0 is open and 1 is closed.
        pos = self.desiredGripperPosition / 100
        # pos = round(pos, 1)
        print('Gripper finger position [0: open, 1: close]: ' + str(pos))
        self.send_gripper_command_func(pos, 'pos')

if __name__ == '__main__':
    gd = GripperDriver()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        gd.setGripperPosition()
        rate.sleep()

    gd.send_gripper_command_func(0, 'pos')  # Fully open the gripper before closing this node.
