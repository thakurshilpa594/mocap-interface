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

class ForceDataCollector:
    def __init__(self, directory, fileName):
        
        self.gripperForces = np.zeros(7) #6 indivdual forces and also the sum
        self.gripperPosition = 0

        rospy.Subscriber('/gripper_sensors', Float32MultiArray, self.forceSensorCB)
        rospy.Subscriber("/my_gen3/base_feedback", BaseCyclic_Feedback, self.baseFeedbackCB)

        # ros spin rate
        self.rate = rospy.Rate(10)
        self.startTime = time.time()
        timeNow   = datetime.datetime.now()
        timestamp = '_' + str(timeNow.year) + '-' + str(timeNow.month).zfill(2) + '-' + str(timeNow.day).zfill(2) + \
                    '_' + str(timeNow.hour).zfill(2) + ':' + str(timeNow.minute).zfill(2)
        

        #self._fileCSV  = open(directory + '/' + fileName + timestamp + '.csv', 'w')
        self._fileCSV  = open(directory + '/' + fileName + '.csv', 'w')
        self._fileCSV.write("Time" + ',' + "Sensor1" + ',' + "Sensor2" + ',' + "Sensor3" + ',' + "Sensor4" + ',' + "Force_Sum" + ',' + "Gripper_Position" + '\n')
        rospy.on_shutdown(self._onShutdown)

    def _onShutdown(self):

        currentTime = time.time() - self.startTime
        self._fileCSV.close()

        print('File closed...')
        print('Run time: ' + str(np.round(currentTime, 2)) + 's')
        print

    def baseFeedbackCB(self, msg):
        self.gripperPosition = msg.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position

    def forceSensorCB(self, msg):
        self.gripperForces[0] = msg.data[0]
        self.gripperForces[1] = msg.data[1]
        self.gripperForces[2] = msg.data[2]
        self.gripperForces[3] = msg.data[3]
        self.gripperForces[4] = msg.data[4]
        self.gripperForces[5] = msg.data[5]
        self.gripperForces[6] = msg.data[0] + msg.data[1] + msg.data[3] + msg.data[4]

    def recordData(self):

        currentTime = (time.time()) - self.startTime
        
        forceData = str(self.gripperForces[0]) + ',' + \
                    str(self.gripperForces[1]) + ',' + \
                    str(self.gripperForces[3]) + ',' + \
                    str(self.gripperForces[4]) + ',' + \
                    str(self.gripperForces[6]) + ',' + \
                    str(self.gripperPosition)

        
        
        self._fileCSV.write(str(currentTime) + ',' + str(forceData) + '\n')


def main():

    fileName  = 'data'
    # directory = '~/data_collection'
    script_dir = os.path.dirname(os.path.abspath(__file__)) #<-- absolute dir the script is in
    rel_path = "/data_files/spongeTest2/Dan"
    directory = script_dir + rel_path   

    print(script_dir)
    print(directory)

    #directory = '/home/rrameshwar/srl-teleoperation/src/mocap-interface/scripts/data_scripts/data_files'

    # get input arguments
    for i in range(len(sys.argv)):

        if sys.argv[i] == '-f' and len(sys.argv) >= i:
            fileName = sys.argv[i+1]
            fileName = fileName
        
        elif sys.argv[i] == '-d' and len(sys.argv) >= i:
            directory = sys.argv[i+1]


    rospy.init_node('collect_force_data')
    node = ForceDataCollector(directory, fileName)

    print('Collect Experiment Data\n')

    while not rospy.is_shutdown():
        
        node.rate.sleep()
        node.recordData()


if __name__ == "__main__":
    main()
