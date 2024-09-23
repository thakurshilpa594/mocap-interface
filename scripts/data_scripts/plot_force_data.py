#!/usr/bin/env python

import rospy
import numpy as np
import sys
import time
import datetime
import matplotlib.pyplot as plt
import pandas as pd
import os

def forceConvert(readings):
	for i in range(0, len(readings)):
		readings[i] = (readings[i]**2 * -0.236 + 24.86*readings[i] + 7.11) * 9.8

	return readings


def main():

	fileName  = 'data.csv'
	# directory = '~/data_collection'
	script_dir = os.path.dirname(os.path.abspath(__file__)) #<-- absolute dir the script is in
	rel_path = "/data_files/spongeTest2/Ritwik"
	directory = script_dir + rel_path   

	startPoint = 0
	endPoint = 0

	#print(script_dir)
	#print(directory)

	#directory = '/home/rrameshwar/srl-teleoperation/src/mocap-interface/scripts/data_scripts/data_files'

	# get input arguments
	for i in range(len(sys.argv)):

		if sys.argv[i] == '-f' and len(sys.argv) >= i:
			fileName = sys.argv[i+1]
		
		elif sys.argv[i] == '-d' and len(sys.argv) >= i:
			directory = sys.argv[i+1]

		elif sys.argv[i] == '-s' and len(sys.argv) >= i:
			startPoint = int(sys.argv[i+1])
			print("startpoint is ", startPoint)

		elif sys.argv[i] == '-e' and len(sys.argv) >= i:
			endPoint = int(sys.argv[i+1])
			print("endpoint is ", endPoint)


	to_open = directory + '/' + fileName
	print("OPENING", to_open)



	plt.rcParams["figure.figsize"] = [7.00, 3.50]
	plt.rcParams["figure.autolayout"] = True
	
	columns = ["Time", "Sensor1", "Sensor2", "Sensor3", "Sensor4", "Force_Sum", "Gripper_Position"]
	
	df = pd.read_csv(to_open, usecols=columns)

	if endPoint == 0:
		endPoint = len(df.Sensor1)
	print(len(df.Sensor1))
	
	#print("Contents in csv file:\n", df)
	# plt.plot(df.Time[startPoint:endPoint], df.GripperTip[startPoint:endPoint], linewidth=2.0)
	
	#plt.plot(df.Time[startPoint:endPoint], forceConvert(df.Normal_A[startPoint:endPoint]), linewidth=2.0)
	#plt.plot(df.Time[startPoint:endPoint], forceConvert(df.Normal_B[startPoint:endPoint]), linewidth=2.0)

	plt.plot(df.Time[startPoint:endPoint], df.Sensor1[startPoint:endPoint], linewidth=2.0)
	plt.plot(df.Time[startPoint:endPoint], df.Sensor2[startPoint:endPoint], linewidth=2.0)

	plt.plot(df.Time[startPoint:endPoint], df.Sensor3[startPoint:endPoint], linewidth=2.0)
	plt.plot(df.Time[startPoint:endPoint], df.Sensor4[startPoint:endPoint], linewidth=2.0)
	plt.plot(df.Time[startPoint:endPoint], df.Force_Sum[startPoint:endPoint], linewidth=2.0)
	# plt.plot(df.Time[startPoint:endPoint], df.Normal_E[startPoint:endPoint]/1000, linewidth=2.0, label='Normal Force')
	
	plt.legend(fontsize=12)
	plt.xlabel("Time (s)", fontsize=14)
	#plt.ylim([-0.65, .650])
	plt.ylabel("Applied Force (N)", fontsize=14)

	plt.show()

	all_forces = df['Force_Sum'].values.tolist()

	force_maxes = []

	for i in range(0, 5):
		m = max(all_forces)
		force_maxes.append(m)
		
		all_forces = [value for value in all_forces if value != m]


	force_maxes.append(np.average(force_maxes))

	print(force_maxes)

if __name__ == '__main__':
	main()


