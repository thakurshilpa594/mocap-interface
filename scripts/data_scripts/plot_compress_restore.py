#!/usr/bin/env python

import rospy
import numpy as np
import sys
import time
import datetime
import matplotlib.pyplot as plt
import pandas as pd
import os

import numpy as np
from scipy.signal import butter,filtfilt# Filter requirements.

T = 5.0         # Sample Period
fs = 30.0       # sample rate, Hz
cutoff = 1.2      # desired cutoff frequency of the filter, Hz ,      slightly higher than actual 1.2 Hznyq = 0.5 * fs  # Nyquist Frequencyorder = 2       # sin wave can be approx represented as quadratic
n = int(T * fs) # total number of samples
nyq = 0.5 * fs  # Nyquist Frequencyorder = 2    
order = 2


def butter_lowpass_filter(data, cutoff, fs, order):
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y


if __name__ == '__main__':
	filename = "data_files/compression_restore_test.csv"
	script_dir = os.path.dirname(os.path.abspath(__file__)) #<-- absolute dir the script is in


	plt.rcParams["figure.figsize"] = [7.00, 3.50]
	plt.rcParams["figure.autolayout"] = True
	
	columns = ["PWM", "Plastic_Compression", "Plastic_Restoration", "Fabric_Compression", "Fabric_Restoration"]

	to_open = script_dir + '/' + filename
	
	df = pd.read_csv(to_open, usecols=columns)

	startPoint = 513
	endPoint = 513 + 255

	plastic_comp_vals = df.Plastic_Compression[startPoint:endPoint].tolist()
	plastic_comp_vals = [x - plastic_comp_vals[0] for x in plastic_comp_vals]
	
	fabric_comp_vals = df.Fabric_Compression[startPoint:endPoint].tolist()
	fabric_comp_vals = [x - fabric_comp_vals[0] for x in fabric_comp_vals]

	pwm_duty_cycle = df.PWM[startPoint:endPoint].tolist()

	pwm_duty_cycle = [x*100/255 for x in pwm_duty_cycle]

	plastic_filt = butter_lowpass_filter(plastic_comp_vals, cutoff, fs, order)
	fabric_filt = butter_lowpass_filter(fabric_comp_vals, cutoff, fs, order)

	plastic_rest_vals = df.Plastic_Restoration[0:255].tolist()
	plastic_rest_vals = [x - plastic_rest_vals[0] for x in plastic_rest_vals]

	fabric_rest_vals = df.Fabric_Restoration[startPoint:endPoint].tolist()
	fabric_rest_vals = [x - fabric_rest_vals[0] for x in fabric_rest_vals]





	plt.plot(pwm_duty_cycle, plastic_rest_vals, linewidth=2.0, color='orange', label='Plastic_Restoration')
	plt.plot(pwm_duty_cycle, fabric_rest_vals, linewidth=2.0, color='blue', label='Ecoflex0030_Restoration')
	plt.legend(fontsize=12)
	plt.xlabel("PWM (% Duty Cycle)", fontsize=14)
	#plt.ylim([-0.65, .650])
	plt.ylabel("Applied Force (N)", fontsize=14)

	plt.figure()


	plt.plot(pwm_duty_cycle, plastic_filt/50, linewidth=2.0, label="Plastic_Compression", color='orange')
	plt.plot(pwm_duty_cycle, fabric_filt/50, linewidth=2.0, label="Ecoflex0030_Compression", color='blue')
	plt.legend(fontsize=12)
	plt.xlabel("PWM (% Duty Cycle)", fontsize=14)
	#plt.ylim([-0.65, .650])
	plt.ylabel("Applied Force (N)", fontsize=14)

	plt.show()