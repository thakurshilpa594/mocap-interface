#!/usr/bin/env python

import roslib
import rospy
import serial
import time

from std_msgs.msg import Int16

if __name__ == '__main__':

	rospy.init_node('gripper_sensor_driver')
	rate = rospy.Rate(100)

	ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0, parity='N', rtscts=1)
	print(ser.name)

	dataRequestMessage = 1

	sensor_pub = rospy.Publisher("/gripper_sensors", Int16, queue_size=1000)
	
	sensor_msg = Int16()

	while not rospy.is_shutdown():
		ser.write('100')
		
		time.sleep(0.05)

		serData = ser.readline()
		print(serData)
		dataLen = len(serData)
		print(dataLen)

		#if dataLen:
			#dataIntegrity = unicode(serData[0:-1]).isnumeric()
			#print(dataIntegrity)
			#sensor_msg.data = int(serData[0:-1])
			#sensor_pub.publish(sensor_msg)

		rate.sleep()

	ser.close()

