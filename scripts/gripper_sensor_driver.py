#!/usr/bin/env python

import roslib
import rospy

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

import bluetooth

def interpretBuffer(buff):
    buff_arr = buff.split(',')
    buff_floats = []
    for i in buff_arr:
        buff_floats.append(float(i))
    
    return buff_floats

if __name__ == '__main__':

	rospy.init_node('gripper_sensor_driver')
	rate = rospy.Rate(100)

	sensor_address = '00:21:08:35:1A:D4'
	socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
	#socket.settimeout(10)

	print("trying to connect")
	socket.connect((sensor_address, 1))
	#socket.create_connection(sensor_address, timeout=10)

	#socket.settimeout(None)

	print("Connected to transmitter")

	sensor_pub = rospy.Publisher("/gripper_sensors_raw", Float32MultiArray, queue_size=1000)
	sensor_msg = Float32MultiArray()

	buffer = ""
	collect = False

	while not rospy.is_shutdown():
	    data = socket.recv(1024)

	    for i in data:
	    	#print(i)
	        if (i == 'a' and len(buffer) == 0):
	            collect = True
	        
	        if (i == 'z' and len(buffer) > 0):
	            buff_floats = interpretBuffer(buffer)
	            for i in buff_floats:
	            	sensor_msg.data.append(i)
	            sensor_pub.publish(sensor_msg)
	            sensor_msg.data = []
	            collect = False
	            buffer = ""
	        
	        if (collect and not i=='a'):
	            buffer += i
	    #rate.sleep()

	socket.close()

	 