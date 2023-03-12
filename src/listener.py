#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo(data.ranges)
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/front/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()