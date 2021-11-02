#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
# ROS
import rospy
import std_msgs.msg

import os

from config import configs
from stream import Camera

class gStreamer:

    def __init__(self, ENDPOINT, CAM_TYPE):

        # Init Basic variables
        self.name = ''
        self.value = 0
        self.appsrcIsFull = True
        self.connectedOnline = False
        self.ENDPOINT = ENDPOINT
        self.CAM_TYPE = CAM_TYPE

        print("gStreamer: endPoint "+self.ENDPOINT+" selected")
        print("gStreamer: camera "+self.CAM_TYPE+" selected")

        # Init Gstreamer
        # Gst.init(None)
        # Gst.debug_set_active(True)
        os.environ['GST_DEBUG_DUMP_DOT_DIR'] = configs.GRAPHVIZ_PATH

        self.gst_pipes = Camera(ENDPOINT, CAM_TYPE) # Start the main gstreamer pipeline


        # ROS Publisher setup
        rospy.init_node('gstreamer', anonymous=True)

        # stream to a local platform
        self.subLocalStart = rospy.Subscriber('gstreamer/local/start', std_msgs.msg.Bool, self.cbLocalStart)
        self.subLocalStop = rospy.Subscriber('gstreamer/local/stop', std_msgs.msg.Bool, self.cbLocalStop)

        # stream to an online platform
        self.subOnlineStart = rospy.Subscriber('gstreamer/online/start', std_msgs.msg.Int32, self.cbOnlineStart)
        self.subOnlineStop = rospy.Subscriber('gstreamer/online/stop', std_msgs.msg.Bool, self.cbOnlineStop)

        # store to file
        self.subStopRecording = rospy.Subscriber('gstreamer/recording/start', std_msgs.msg.Bool, self.cbStartRecording)
        self.subStartRecording = rospy.Subscriber('gstreamer/recording/stop', std_msgs.msg.Bool, self.cbStopRecording)

        self.subStopRecording = rospy.Subscriber('gstreamer/photo', std_msgs.msg.Bool, self.cbPhoto)

        self.subThermal = rospy.Subscriber('gstreamer/thermal', std_msgs.msg.Bool, self.cbThermal)

        self.rate = rospy.Rate(2)


    def cbLocalStart(self, msg):
        print("Start local stream")
        self.gst_pipes.startPipeT1()

    def cbLocalStop(self, msg):
        print("Stop local stream")
        self.gst_pipes.pausePipeT1()

    def cbOnlineStart(self, msg):
        print("Enter online mode on port "+str(msg.data))
        self.gst_pipes.startOnlineStream(configs.JANUS_LIST[self.ENDPOINT], msg.data)

    def cbOnlineStop(self, msg):
        print("Stop online stream")
        self.gst_pipes.stopOnlineStream()

    def cbStartRecording(self, msg):
        print("START RECORDING")
        self.gst_pipes.startRecord()
        
    def cbStopRecording(self, msg):
        print("STOP RECORDING")
        self.gst_pipes.stopRecord()

    def cbPhoto(self, msg):
        # TODO - Change camera configurations
        print("Take photo")
        self.gst_pipes.takePhoto()

    def cbThermal(self, msg):
        if msg.data == 0:
            print("Switching to main stream")
        else:
            print("Switching to thermal stream")

        self.gst_pipes.switchThermal(msg.data)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
