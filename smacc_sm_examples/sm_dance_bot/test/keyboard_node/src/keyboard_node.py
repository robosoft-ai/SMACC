#!/usr/bin/env python

from __future__ import print_function

import roslib;
import rospy
import std_msgs
from std_msgs.msg import UInt16

import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('keyboard_unicode', UInt16, queue_size = 1)
    rospy.init_node('keyboard_node')

    try:
        while(1):
            key = getKey()
            #rospy.loginfo(type(key))
            msg = UInt16()
            msg.data = ord(key)
            
            pub.publish(msg)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)