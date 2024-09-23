#!/usr/bin/env python

import rospy
import numpy as np
import sys
import time
import datetime
from std_msgs.msg import Float32MultiArray
import os

from kortex_driver.srv import *
from kortex_driver.msg import *

class GloveDataCollector:
    def __init__(self, directory, fileName):
        
        self.fingersPositions = np.zeros(5) #5
        rospy.Subscriber('/fingersPos', Float32MultiArray, self.fingersPosCallback)

        # ros spin rate
        self.rate = rospy.Rate(10)
        self.startTime = time.time()
        timeNow   = datetime.datetime.now()
        timestamp = '_' + str(timeNow.year) + '-' + str(timeNow.month).zfill(2) + '-' + str(timeNow.day).zfill(2) + \
                    '_' + str(timeNow.hour).zfill(2) + ':' + str(timeNow.minute).zfill(2)
        

        #self._fileCSV  = open(directory + '/' + fileName + timestamp + '.csv', 'w')
        self._fileCSV  = open(directory + '/' + fileName + '.csv', 'w')
        self._fileCSV.write("Time" + ',' + "Thumb" + ',' + "Index" + ',' + "Middle" + ',' + "Ring" + ',' + "Pinky" + '\n')
        rospy.on_shutdown(self._onShutdown)

    def _onShutdown(self):

        currentTime = time.time() - self.startTime
        self._fileCSV.close()

        print('File closed...')
        print('Run time: ' + str(np.round(currentTime, 2)) + 's')
        print

    def fingersPosCallback(self, msg):
        self.fingersPositions[0] = msg.data[0]
        self.fingersPositions[1] = msg.data[1]
        self.fingersPositions[2] = msg.data[2]
        self.fingersPositions[3] = msg.data[3]
        self.fingersPositions[4] = msg.data[4]

    def recordData(self):

        currentTime = (time.time()) - self.startTime
        
        fingersData = str(self.fingersPositions[0]) + ',' + \
                    str(self.fingersPositions[1]) + ',' + \
                    str(self.fingersPositions[2]) + ',' + \
                    str(self.fingersPositions[3]) + ',' + \
                    str(self.fingersPositions[4])

        
        
        self._fileCSV.write(str(currentTime) + ',' + str(fingersData) + '\n')


def main():

    fileName  = 'data'
    # directory = '~/data_collection'
    script_dir = os.path.dirname(os.path.abspath(__file__)) #<-- absolute dir the script is in
    rel_path = "/data_files/cross_talk_tests/"
    directory = script_dir + rel_path   

    print(script_dir)
    print(directory)

    #directory = '/home/rrameshwar/srl-teleoperation/src/mocap-interface/scripts/data_scripts/data_files'

    # get input arguments
    for i in range(len(sys.argv)):

        if sys.argv[i] == '-f' and len(sys.argv) >= i:
            fileName = sys.argv[i+1]
        
        elif sys.argv[i] == '-d' and len(sys.argv) >= i:
            directory = sys.argv[i+1]


    rospy.init_node('collect_force_data')
    node = GloveDataCollector(directory, fileName)

    print('Collect Experiment Data\n')

    while not rospy.is_shutdown():
        
        node.rate.sleep()
        node.recordData()


if __name__ == "__main__":
    main()
