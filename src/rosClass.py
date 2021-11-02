#!/usr/bin/env python

# ROS
import rospy
import sensor_msgs.msg

class rosClass:
    def __init__(self):
        rospy.init_node('gstreamer', anonymous=True)
        self.rate = rospy.Rate(30)
