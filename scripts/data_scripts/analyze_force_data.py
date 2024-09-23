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


def getFileData(to_open):
	
	columns = ["Time", "Sensor1", "Sensor2", "Sensor3", "Sensor4", "Force_Sum", "Gripper_Position"]
	
	df = pd.read_csv(to_open, usecols=columns)

	all_forces = df['Force_Sum'].values.tolist()

	all_forces = forceConvert(all_forces)

	all_forces = [x - all_forces[0] for x in all_forces]

	force_maxes = []

	for i in range(0, 10):
		m = max(all_forces)
		force_maxes.append(m)
		
		all_forces = [value for value in all_forces if value != m]


	force_max_final = np.average(force_maxes)

	return force_max_final

if __name__ == '__main__':

	fileName  = 'data.csv'
	# directory = '~/data_collection'
	script_dir = os.path.dirname(os.path.abspath(__file__)) #<-- absolute dir the script is in
	rel_path = "/data_files/spongeTest2/"
	directory = script_dir + rel_path   
	

	noHaptic_Maxes = []
	HapticFS_Maxes = []
	HapticP_Maxes = []
	
	users = ["Shilpa", "Gabby", "Ritwik", "Joseph", "Mason", "Raagini", "ShouShan", "Steven", "Dan"]

	for i in range(0, len(users)):
		userfolder = directory + users[i] + "/"

		for j in range(0, 5):
			test = "NoHaptic" + str(j+1) + ".csv"
			filename = userfolder+test
			try:
				force_max = getFileData(filename)
				noHaptic_Maxes.append(force_max)
			except:
				print(filename)

		for j in range(0, 5):
			test = "HapticP" + str(j+1) + ".csv"
			filename = userfolder+test
			try:
				force_max = getFileData(filename)
				HapticP_Maxes.append(force_max)
			except:
				print(filename)

		for j in range(0, 5):
			test = "HapticFS" + str(j+1) + ".csv"
			filename = userfolder+test
			try:
				force_max = getFileData(filename)
				HapticFS_Maxes.append(force_max)
			except:
				print(filename)

	print("NO HAPTICS: ", np.average(noHaptic_Maxes))
	print("PLASTIC HAPTICS: ", np.average(HapticP_Maxes))
	print("FABRIC HAPTICS: ", np.average(HapticFS_Maxes))
	



