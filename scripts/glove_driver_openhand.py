#!/usr/bin/env python

# Basic glove driver that collects finger position data from the USB dongle and publishes it to the topic "fingersPos".

import rospy
import serial
import numpy as np

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

import hands

# Disable scientific notation.
np.set_printoptions(suppress=True)

if __name__ == '__main__':

    rospy.init_node('glove_driver')

    T = hands.Model_O('/dev/ttyUSB0', 1, 4, 3, 2, "XM", 0.0, 0.2, 0.02, 0.47)

    pub_fingers = rospy.Publisher("fingersPos", Float32MultiArray, queue_size=1000)

    # for i in range(0, 16):
    #     glove_array.append([])

    rate = rospy.Rate(100)

    def send_motor_commands(event):
        global array

        thumb_value = round(np.interp(
                array[1],
                [
                    191.0,
                    197.0 - 2,
                ],
                [
                    0.0,
                    0.6,
                ],
            ), 3)
        index_value = round(np.interp(
            array[5],
            [
                188.0,
                192.0 - 2,
            ],
            [
                0.0,
                0.6,
            ],
        ), 3)
        middle_value = round(
            np.interp(
                array[8],
                [
                    187.0,
                    192.0 - 2,
                ],
                [
                    0.0,
                    0.6,
                ],
            ),
            3)

        T.moveMotor(1, index_value)
        T.moveMotor(2, middle_value)
        T.moveMotor(3, thumb_value)
        # print(f'{thumb_value=} {index_value=} {middle_value=}')

    # # Timers:
    rospy.Timer(
        rospy.Duration(1.0 / 10),
        send_motor_commands,
    )





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
                # print(msg_fingers.data)
                # print(round(array[1], 2), round(array[5], 2), round(array[8], 2))
                pub_fingers.publish(msg_fingers)



                # rospy.loginfo_throttle(1, f'{round(array[1], 2)=} {round(array[5], 2)=} {round(array[8], 2)=}')
            rate.sleep()
