#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import String #String message 
from std_msgs.msg import Int8
import sys

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

from select import select


################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################


# def keys():
#     pub = rospy.Publisher('key',Int8,queue_size=10) # "key" is the publisher name
#     rospy.init_node('keypress',anonymous=True)
#     rate = rospy.Rate(10)#try removing this line ans see what happens
#     while not rospy.is_shutdown():
#         k=ord(getch.getch())# this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
#         if ((k>=65)&(k<=68)|(k==115)|(k==113)|(k==97)):# to filter only the up , dowm ,left , right key /// this line can be removed or more key can be added to this
#             rospy.loginfo(str(k))# to print on  terminal 
#             pub.publish(k)#to publish

        #rospy.loginfo(str(k))

        #rate.sleep()

#s=115,e=101,g=103,b=98

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=='__main__':
    pub = rospy.Publisher('key',String,queue_size=10) # "key" is the publisher name
    rospy.init_node('keypress',anonymous=True)
    rate = rospy.Rate(10)
    settings = saveTerminalSettings()

    while not rospy.is_shutdown():
        k = getKey(settings, 0.5)
        pub.publish(k)