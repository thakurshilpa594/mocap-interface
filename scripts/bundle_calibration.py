#!/usr/bin/env python

import rospy
import numpy as np
import sys

from tf.msg                  import tfMessage
from scipy.spatial.transform import Rotation as r

# disable scientific notation
np.set_printoptions(suppress=True)

class Calibrate:

    def __init__(self):

        self.isComplete = False
        self._numIterations = 100

        self.masterTagName = ""
        self._masterTagTransform = (r.from_euler('xyz', (0.0, 0.0, 0.0)), np.zeros(3))
        self._childTagsTransform = {}
        self._history            = []

        # ros spin rate
        self.rate = rospy.Rate(100)
        
        # subscribers
        rospy.Subscriber('/tf', tfMessage, self._callbackTF)
    
    # get apriltag transforms and calculate error
    def _callbackTF(self, dataTFMessage):

        print("------------------------------------------")

        # get AprilTag transforms
        for det in dataTFMessage.transforms:

            rot = r.from_quat((det.transform.rotation.x, det.transform.rotation.y,
                                    det.transform.rotation.z, det.transform.rotation.w))
            trn = np.array((det.transform.translation.x, det.transform.translation.y,
                                    det.transform.translation.z))

            # check for master tag
            if det.child_frame_id == self.masterTagName:
                self._masterTagTransform = (rot, trn)

            else:
                self._childTagsTransform[det.child_frame_id] = (rot, trn)

        print(self.masterTagName)
        print(self._masterTagTransform[0].as_quat().round(6))
        print(self._masterTagTransform[1].round(3))
        print("------------------------------------------")

        # build master transform
        transformMaster = np.vstack((np.hstack((self._masterTagTransform[0].as_dcm(),
                                    np.reshape(self._masterTagTransform[1], (3, 1)))), np.array((0, 0, 0, 1))))
        
        # get offsets
        for name, transform in self._childTagsTransform.iteritems():

            # build child transform
            transformChild = np.vstack((np.hstack((transform[0].as_dcm(),
                                    np.reshape(transform[1], (3, 1)))), np.array((0, 0, 0, 1))))
            
            # find offset between child and master
            tMasterToChild = np.matmul(np.linalg.inv(transformMaster), transformChild)
            pMasterToChild = tMasterToChild[:3, 3].T
            rMasterToChild = r.from_dcm(tMasterToChild[:3, :3])

            # add to history
            self._history.append((name, rMasterToChild, pMasterToChild))

            print(name)
            print(rMasterToChild.as_quat().round(6))
            print(pMasterToChild.round(3))
            print("------------------------------------------")
        
        print
        print

        # check for completion
        if len(self._history) >= self._numIterations*len(self._childTagsTransform):

            self._history = self._history[len(self._childTagsTransform)-1:]
            self.isComplete = True
            self._averageHistory()
    
    def _averageHistory(self):

        # initialize dictionary
        childTransformMeans  = {}
        for name, transform in self._childTagsTransform.iteritems():

            childTransformMeans[name] = [np.zeros(3), np.zeros(3)]
        
        # average transform components
        for frame in self._history:
            
            childTransformMeans[frame[0]][0] += frame[1].as_euler('xyz')/self._numIterations
            childTransformMeans[frame[0]][1] += frame[2]/self._numIterations
        
        print("RESULTS ----------------------------------")

        # print averages
        for name, average in childTransformMeans.iteritems():

            rot = r.from_euler('xyz', childTransformMeans[name][0]).as_quat().round(6)
            trn = childTransformMeans[name][1].round(3)

            print(name)
            print('qw: ' + str(rot[3]) + ', qx: ' + str(rot[0]) + ', qy: ' + str(rot[1]) + ', qz: ' + str(rot[2]))
            print('x: ' + str(trn[0]) + ', y: ' + str(trn[1]) + ', z: ' + str(trn[2]))
            print("------------------------------------------")
        
        print
        print

def main():

    rospy.init_node('bundle_calibration')
    node = Calibrate()
    node.masterTagName = sys.argv[1] if len(sys.argv) > 1 else ""

    print('AprilTag Bundle Calibration\n')

    while not rospy.is_shutdown() and not node.isComplete:
        
        node.rate.sleep()


if __name__ == "__main__":
    main()
