#!/usr/bin/env python
import rospy
import numpy

from sensor_msgs.msg import BatteryState

if __name__ == "__main__":
    rospy.init_node("battery_node")
    pub = rospy.Publisher("battery", BatteryState, queue_size=1)
    tinit = rospy.Time.now().to_time()
    speed = 0.05

    try:
        while not rospy.is_shutdown():
            msg = BatteryState()

            t = rospy.Time.now().to_time()
            msg.charge = 10 * numpy.exp(speed * (tinit - t)) - 1.0

            rospy.sleep(0.5)
            pub.publish(msg)

    except Exception as e:
        print(e)
