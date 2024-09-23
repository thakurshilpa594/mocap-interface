#!/usr/bin/env python

# Basic glove driver that collects finger position data from the USB dongle and publishes it to the topic "fingersPos".

import rospy
import serial
import numpy as np

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

# Disable scientific notation.
np.set_printoptions(suppress=True)

if __name__ == '__main__':

    rospy.init_node('glove_driver')

    pub_fingers = rospy.Publisher("fingersPos1", Float32MultiArray, queue_size=1000)

    # for i in range(0, 16):
    #     glove_array.append([])

    rate = rospy.Rate(100)

    msg_fingers = Float32MultiArray()

    with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
        while not rospy.is_shutdown():
            line = ser.readline()  # Read a '\n' terminated line of bytes.
            decodedLine = line.decode()  # A method to convert bytes into a string.
            array = decodedLine.split(',', 30)[0:16]
            # print(array)

            if len(array) >= 16:

                for i in range(0, len(array)):
                    array[i] = float(array[i][2:])

                msg_fingers.data = [round(array[1], 2), round(array[5], 2), round(array[8], 2), round(array[11],2), round(array[14],2)]
                #msg_fingers.data = [
                    #(array[0] + 10*array[1] + 100*array[2]),
                    #(array[3] + 10*array[4] + 100*array[5]),
                    #(array[6] + 10*array[7] + 100*array[8]),
                    #(array[9] * 10*array[10] + 100*array[11]),
                    #(array[12] + 10*array[13] + 100*array[14])
                    #]

                print(msg_fingers.data)
                pub_fingers.publish(msg_fingers)

            rate.sleep()
