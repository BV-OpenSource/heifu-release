#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@author: João Pedro Carvalho
"""
from vidgear.gears import VideoGear
import numpy as np
import cv2
import argparse

# Argument parser
parser = argparse.ArgumentParser(description='Play any stream video with stabiliza.')
parser.add_argument("-p", "--port",    type=int,   default=50193,         help='Port janus')
parser.add_argument("-v", "--version",         help="show program version",              action="store_true")

args = parser.parse_args()

if args.version:
    print('Version 0.1.')
    exit()

'''
GST initialization
'''
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)
Gst.debug_set_active(True)

pipe = Gst.Pipeline.new('dynamic')
src = Gst.ElementFactory.make('appsrc')
encoder = Gst.ElementFactory.make('x264enc')
payloader = Gst.ElementFactory.make('rtph264pay')
sink = Gst.ElementFactory.make('multiudpsink')
videoconv = Gst.ElementFactory.make('videoconvert')
videoscale = Gst.ElementFactory.make('videoscale')

encoder.set_property('tune', 'zerolatency')
encoder.set_property('pass', 'qual')
encoder.set_property('quantizer', 20)
encoder.set_property('bitrate', 10000)
encoder.set_property('key-int-max', 1)
caps = Gst.Caps.from_string('video/x-h264, profile=(string)constrained-baseline ')
filter = Gst.ElementFactory.make('capsfilter')
filter.set_property('caps', caps)
src.set_property('format', Gst.Format.TIME)
sink.set_property('clients', 'janus.preprod.pdmfc.com:' + str(args.port))
sink.set_property('sync', False)
src.set_property('caps', Gst.Caps.from_string('video/x-raw, width=640, height=360, framerate=30/1, format=(string)BGR '
                                              ))
payloader.set_property('config-interval', 1)

pipe.add(src)
pipe.add(encoder)
pipe.add(filter)
pipe.add(payloader)
pipe.add(sink)
pipe.add(videoconv)
pipe.add(videoscale)

src.link(videoscale)
videoscale.link(videoconv)
videoconv.link(encoder)
encoder.link(filter)
filter.link(payloader)
payloader.link(sink)

bus = pipe.get_bus()
pipe.set_state(Gst.State.PLAYING)

# def

duration = (1.0 / 30.0) * Gst.SECOND
num_frames = 0

# open any valid video stream with stabilization enabled(`stabilize = True`)
stream_stab = VideoGear(source=0, stabilize=True).start()

dx = 10
dy = 10
# loop over
while True:

    # read stabilized frames
    vectorImageDxDy = stream_stab.read()

    if vectorImageDxDy is None:
        break

    frame_stab = vectorImageDxDy[0]  # read stabilize image

    if int(abs(vectorImageDxDy[1])) > dx:
        dx = int(abs(vectorImageDxDy[1])) + 5  # read dx translation
        # print('Change dx to-> %d' % dx)
    if int(abs(vectorImageDxDy[2])) > dy:
        dy = int(abs(vectorImageDxDy[2])) + 5  # read dy translation
        # print('Change dy to-> %d' % dy)

    h, w, _ = frame_stab.shape
    dx = 0
    dy = 0
    crop_img = frame_stab[dy:h - dy, dx:w - dx]

    dim = (w, h)
    resized = cv2.resize(crop_img, dim, interpolation=cv2.INTER_AREA)

    timestamp = num_frames * duration
    gst_buffer = Gst.Buffer.new_wrapped(resized.tobytes())
    gst_buffer.pts = timestamp
    gst_buffer.dts = timestamp
    gst_buffer.duration = duration
    src.emit("push-buffer", gst_buffer)
    num_frames += 1

# close output window
cv2.destroyAllWindows()

# safely close both video streams
stream_stab.stop()
pipe.set_state(Gst.State.NULL)
