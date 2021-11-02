#!/usr/bin/env python

# if gstreamer fails to create capture session:
# sudo systemctl restart nvargus-daemon

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp

import numpy as np
import cv2


# Add many elements to pipe
def addMany(pipe, *args):
    for arg in args:
        arg, pipe.add(arg)
    return pipe

# Link many elements to pipe and check for errors
def linkMany(*args):
    print(args)
    i = 0
    while i < (len(args)-1):
        i += 1
        link_ret = args[i-1].link(args[i])
        if link_ret not in {None, True}:
            print("!! Error linking: Link "+str(i)+" status (False=Fail): "+str(link_ret))
            #exit(1)
            pass


# Init gst
Gst.init(None)
Gst.debug_set_active(True)

#rtspsrc location=rtsp://heifu:heifu@192.168.42.108:554/cam/realmonitor?channel=1&subtype="+str(subType)+" latency=100 ! rtph264depay ! avdec_h264 ! videoconvert name=conv"

'''
# ERROR: Cant build pipe manually - rtspsrc fails to link second element! -> build first part of pipe with parse_launch
pipe = Gst.Pipeline.new('main_pipe')

rtspsrc = Gst.ElementFactory.make('rtspsrc')
rtspsrc.set_property("location", "'rtsp://heifu:heifu@192.168.42.108:554/cam/realmonitor?channel=1&subtype=0'")
rtspsrc.set_property("latency", 100)
rtph264depay = Gst.ElementFactory.make('rtph264depay')
avdec_h264 = Gst.ElementFactory.make('avdec_h264')
videoconvert = Gst.ElementFactory.make('videoconvert')
autovideosink = Gst.ElementFactory.make('autovideosink')

addMany(pipe, rtspsrc, rtph264depay, avdec_h264, videoconvert, autovideosink)
linkMany(rtspsrc, rtph264depay, avdec_h264, videoconvert, autovideosink)
'''


pipe_cmd = "rtspsrc location=rtsp://heifu:heifu@192.168.42.108:554/cam/realmonitor?channel=1&subtype=0 latency=100 ! rtph264depay ! avdec_h264 ! videoconvert name=conv"
pipe = Gst.parse_launch(pipe_cmd)
#pipe.set_state(Gst.State.PLAYING)

videoconvert = pipe.get_by_name('conv')
autovideosink = Gst.ElementFactory.make('autovideosink')

addMany(pipe, autovideosink)
linkMany(videoconvert, autovideosink)


bus = pipe.get_bus()
pipe.set_state(Gst.State.PLAYING)

raw_input("Press Enter to continue...")

pipe.set_state(Gst.State.NULL)

