#!/usr/bin/env python

# if gstreamer fails to create capture session:
# sudo systemctl restart nvargus-daemon

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp
import numpy as np
import time
from datetime import datetime

import os, sys
import glob
import subprocess
from pathlib import Path

from config import configs

class Camera:
	REALSENSE = 1
	LAPTOP = 2
	ARDUCAM = 3
	raspy = 4

	def __init__(self, ENDPOINT, CAM_TYPE):
		# Init Basic variables
		self.name = ''
		self.value = 0
		self.appsrcIsFull = True
		self.connectedOnline = False
		self.JETSON = True

		self.ENDPOINT = ENDPOINT
		self.CAM_TYPE = CAM_TYPE

		self.recording_state = "starting"

		# Init Gstreamer
		Gst.init(None)
		Gst.debug_set_active(True)


		print (configs.GRAPHVIZ_PATH)


		try:
			Path(configs.GRAPHVIZ_PATH).mkdir(parents=True, exist_ok=True)
		except: 
			print("GST_PIPES: error creating graphv")



		print("Camera "+self.CAM_TYPE+" selected")

		if self.CAM_TYPE == "insta360":
			print("GST_PIPES: insta360 camera selected - TeePipe not built")
			
		else:
			# Build main pipe and tee
			self._buildPipeline(thermal=0)

			# Start playing main pipe
			self.startTeePipe()

			# Begin local stream
			#self.startPipeT1()

			# Save pipeline graph
			if configs.SAVE_PIPELINES == True:
				self.savePipeline(self.TeePipe, "TeePipe")


	def getName(self):
		return self.name

	def getValue(self):
		return self.value

	def _appsrcIsFullCb(self, source):
		self.appsrcIsFull = True

	def _appsrcNeedDataCb(self, source, length):
		self.appsrcIsFull = False

	def startTeePipe(self):
		self.TeePipe.set_state(Gst.State.PLAYING)
		print("GST_PIPES: TeePipe playing")

	def pauseTeePipe(self):
		self.TeePipe.set_state(Gst.State.Gst.State.PAUSED)
		print("GST_PIPES: TeePipe paused")

	def startPipeT1(self):
		self.begin_pipe_t1()
		self.pipe_t1.set_state(Gst.State.PLAYING)
		print("GST_PIPES: pipe_t1 playing")

	def pausePipeT1(self):
		self.pipe_t1.set_state(Gst.State.PAUSED)
		print("GST_PIPES: pipe_t1 paused")

	def startOnlineStream(self, JANUS_IP, port):
		if self.CAM_TYPE == "insta360":
			print("GST_PIPES: Forwarding insta360 stream to janus..")
			stream_cmd = "gst-launch-1.0 -v rtspsrc location=rtsp://" + str(configs.INSTA360_IP) + "/live/live latency=0 ! multiudpsink clients=" + str(JANUS_IP) + ':' + str(port) + " sync=false"
			stream_process = subprocess.Popen(['bash', '-c', stream_cmd], shell=False)  # Run the process to video recorder

		else:
			self.begin_pipe_t2(JANUS_IP, port)
			self.pipe_t2.set_state(Gst.State.PLAYING)
			print("GST_PIPES: pipe_t2 playing")

	def stopOnlineStream(self):
		self.pipe_t2.set_state(Gst.State.PAUSED)

	# Add many elements to pipe
	def addMany(self, pipe, *args):
		for arg in args:
			arg, pipe.add(arg)
		#return pipe

	# Link many elements to pipe and check for errors
	def linkMany(self, *args):
		i = 0
		while i < (len(args)-1):
			i += 1
			#pre_elem = args[i-1]
			#next_elem = args[i]
			link_ret = args[i-1].link(args[i])
			if link_ret not in {None, True}:
				print("!! Error linking: Link "+str(i)+" status (False=Fail): "+str(link_ret))
				#exit(1)
				pass
			
	def savePipeline(self, pipe, pipe_name):
		pipe_name = str(pipe_name)


		print (configs.GRAPHVIZ_PATH)


		Gst.debug_bin_to_dot_file(pipe, Gst.DebugGraphDetails(15), pipe_name)

		os.system("dot -Tpdf "+configs.GRAPHVIZ_PATH+"/"+pipe_name+".dot > "+configs.GRAPHVIZ_PATH+"/"+pipe_name+".pdf")
		os.system("dot -Tpng "+configs.GRAPHVIZ_PATH+"/"+pipe_name+".dot > "+configs.GRAPHVIZ_PATH+"/"+pipe_name+".png")
		file_list = glob.glob(configs.GRAPHVIZ_PATH+"/*.dot")
		for filePath in file_list:
			try:
				os.remove(filePath)
			except OSError:
				print("Error while deleting file")


	def _buildPipeline(self, thermal=0):

		if thermal not in {0,1}:
			print("_buildPipeline: Incorrect thermal int value")
			exit(1)

		print("is jetson: "+str(configs.JETSON))
		# if camera is eh640 use parse_launch to build first part of TeePipe 
		#   (couldn't link rtspsrc to other elements another way)
		if self.CAM_TYPE == "eh640":
			if thermal == 0: 
				subType = 0
			else: 
				subType = 2
			pc_cmd = "rtspsrc location=rtsp://heifu:heifu@192.168.42.108:554/cam/realmonitor?channel=1&subtype="+str(subType)+"' latency=500 ! rtph264depay ! h264parse ! omxh264dec ! videoconvert name=conv"
			jetson_cmd = "rtspsrc location=rtsp://heifu:heifu@192.168.42.108:554/cam/realmonitor?channel=1&subtype="+str(subType)+" latency=100 ! rtph264depay ! avdec_h264 ! videoconvert name=conv"
			pipe_cmd = jetson_cmd if configs.JETSON else pc_cmd

			self.TeePipe = Gst.parse_launch(pipe_cmd)
			self.TeePipe.set_state(Gst.State.PLAYING)

			main_sink = self.TeePipe.get_by_name('conv')

		else:
			self.TeePipe = Gst.Pipeline.new('main_pipe')

		if configs.JETSON:
			if self.CAM_TYPE == "default":
				src = Gst.ElementFactory.make('nvarguscamerasrc')
				videorate = Gst.ElementFactory.make('videorate')
				filterCaps0 = Gst.ElementFactory.make('capsfilter')
				caps0 = Gst.Caps.from_string('video/x-raw(memory:NVMM), width=(int)' + str(4032) + ', height=(int)' + str(3040) + ', framerate=30/1')
				filterCaps0.set_property('caps', caps0)

				# Crop and scale
				videoconv_gpu_crop = Gst.ElementFactory.make('nvvidconv')
				videoconv_gpu_crop.set_property("top", 440)
				videoconv_gpu_crop.set_property("bottom", 2600)
				videoconv_gpu_crop.set_property("left", 96)
				videoconv_gpu_crop.set_property("right", 3936)
				videoconv_gpu_crop.set_property('flip-method', 0)

				filterCaps1 = Gst.ElementFactory.make('capsfilter')
				caps1 = Gst.Caps.from_string('video/x-raw(memory:NVMM), width=(int)' + str(3840) + ', height=(int)' + str(2160))
				filterCaps1.set_property('caps', caps1)

				videoconv_gpu_0 = Gst.ElementFactory.make('nvvidconv')
				main_sink = videoconv_gpu_0

				self.addMany(self.TeePipe, src, videorate, filterCaps0, videoconv_gpu_crop, filterCaps1, videoconv_gpu_0)

				self.linkMany(src, videorate, filterCaps0, videoconv_gpu_crop, filterCaps1, videoconv_gpu_0)

		else: # not JETSON: gst-launch-1.0 -e autovideosrc ! videoconvert ! ximagesink
			if self.CAM_TYPE == "default":
				src = Gst.ElementFactory.make('v4l2src') # autovideosrc
				videoconv = Gst.ElementFactory.make('videoconvert')
				main_sink = videoconv

				self.addMany(self.TeePipe, src, videoconv)
				self.linkMany(src, videoconv)
			elif self.CAM_TYPE == "gazeebo_integrated":
				self.TeePipe = Gst.Pipeline.new("local-stream")

				#gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
				#autovideosink fps-update-interval=1000 sync=false

				udpsrc = Gst.ElementFactory.make("udpsrc")
				udpsrc.set_property("port", 5600)
				caps = Gst.Caps.from_string('application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264')
				udpsrc.set_property("caps", caps)
				rtpdepay = Gst.ElementFactory.make('rtph264depay')
				avdec = Gst.ElementFactory.make('avdec_h264')
				videoconvert = Gst.ElementFactory.make('videoconvert')
				self.addMany(self.TeePipe, udpsrc, rtpdepay, avdec, videoconvert)
				self.linkMany(udpsrc, rtpdepay, avdec, videoconvert)
				main_sink = videoconvert

			

				
		# Add and link tee elements and intersinks
		self.buildTee(self.TeePipe, main_sink)

		print("GST_PIPES: TeePipe ready")


	def buildTee(self, pipe, input_elem):

		tee = Gst.ElementFactory.make ("tee")

		# Make elements
		# > thread 1
		video_queue1 = Gst.ElementFactory.make ("queue", "video_queue1")
		intervideosink1 = Gst.ElementFactory.make ("intervideosink")
		intervideosink1.set_property("channel", "t1")
		# > thread 2
		video_queue2 = Gst.ElementFactory.make ("queue", "video_queue2")
		intervideosink2 = Gst.ElementFactory.make ("intervideosink")
		intervideosink2.set_property("channel", "t2")
		# > thread 3
		video_queue3 = Gst.ElementFactory.make ("queue", "video_queue3")
		intervideosink3 = Gst.ElementFactory.make ("intervideosink")
		intervideosink3.set_property("channel", "t3")
		# > thread 4
		video_queue4 = Gst.ElementFactory.make ("queue", "video_queue4")
		intervideosink4 = Gst.ElementFactory.make ("intervideosink")
		intervideosink4.set_property("channel", "t4")

		# Add elems to pipe
		self.addMany(pipe, tee, video_queue1, intervideosink1, video_queue2, intervideosink2, video_queue3, intervideosink3, video_queue4, intervideosink4)

		# Link elements
		if not input_elem.link(tee):
			print("#1 Elements could not be linked.")
			sys.exit(1)
		# Link thread 1
		if not video_queue1.link(intervideosink1):
			print("#2 Elements could not be linked.")
			sys.exit(1)
		# Link thread 2
		if not video_queue2.link(intervideosink2):
			print("#3 Elements could not be linked.")
			sys.exit(1)
		 # Link thread 3
		if not video_queue3.link(intervideosink3):
			print("#4 Elements could not be linked.")
			sys.exit(1)
		 # Link thread 4
		if not video_queue4.link(intervideosink4):
			print("#4 Elements could not be linked.")
			sys.exit(1)

		# Multi-thread creation
		tee_video_pad1 = tee.get_request_pad("src_%u")
		queue_video_pad1 = video_queue1.get_static_pad("sink")

		tee_video_pad2 = tee.get_request_pad("src_%u")
		queue_video_pad2 = video_queue2.get_static_pad("sink")

		tee_video_pad3 = tee.get_request_pad("src_%u")
		queue_video_pad3 = video_queue3.get_static_pad("sink")

		tee_video_pad4 = tee.get_request_pad("src_%u")
		queue_video_pad4 = video_queue4.get_static_pad("sink")

		#Link pads to threads
		ret = tee_video_pad1.link(queue_video_pad1)
		if ret != 0:
			print("#5 Elements could not be linked.")
			sys.exit(1)
		ret = tee_video_pad2.link(queue_video_pad2)
		if ret != 0:
			print("#6 Elements could not be linked.")
			sys.exit(1)
		ret = tee_video_pad3.link(queue_video_pad3)
		if ret != 0:
			print("#7 Elements could not be linked.")
			sys.exit(1)
		ret = tee_video_pad4.link(queue_video_pad4)
		if ret != 0:
			print("#7 Elements could not be linked.")
			sys.exit(1)
			

	# Local stream (for HereLink)
	def begin_pipe_t1(self):

		rc_display_resolution = (1280, 720)#(1280, 720)#(1920, 1080)

		self.pipe_t1 = Gst.Pipeline.new("local-stream")

		if configs.JETSON:
			if self.CAM_TYPE == "default":
				intervideosrc1 = Gst.ElementFactory.make ("intervideosrc")
				intervideosrc1.set_property("channel", "t1")
				videoconv_gpu_0 = Gst.ElementFactory.make('nvvidconv')
				filterCaps = Gst.ElementFactory.make('capsfilter')
				caps = Gst.Caps.from_string('video/x-raw(memory:NVMM), width=(int)' + str(rc_display_resolution[0]) + ', height=(int)' + str(rc_display_resolution[1]))
				filterCaps.set_property('caps', caps)
				video_sink = Gst.ElementFactory.make("nvoverlaysink")

				self.addMany(self.pipe_t1, intervideosrc1, videoconv_gpu_0, filterCaps, video_sink)
				self.linkMany(intervideosrc1, videoconv_gpu_0, filterCaps, video_sink)

			elif self.CAM_TYPE == "eh640":
				# gst-launch-1.0 rtpbin name=rtpbin rtspsrc location=rtsp://admin:heifu@192.168.42.108:554/live ! rtph264depay ! decodebin ! nvvidconv ! ximagesink
				intervideosrc1 = Gst.ElementFactory.make ("intervideosrc")
				intervideosrc1.set_property("channel", "t1")
				videoconv = Gst.ElementFactory.make('nvvidconv')
				video_sink = Gst.ElementFactory.make("nvoverlaysink")

				self.addMany(self.pipe_t1, intervideosrc1, videoconv, video_sink)
				self.linkMany(intervideosrc1, videoconv, video_sink)

		else: # not JETSON
			if self.CAM_TYPE == "default":
				intervideosrc1 = Gst.ElementFactory.make ("intervideosrc")
				intervideosrc1.set_property("channel", "t1")
				videoconv = Gst.ElementFactory.make('videoconvert')
				video_sink = Gst.ElementFactory.make("autovideosink")

				self.addMany(self.pipe_t1, intervideosrc1, videoconv, video_sink)
				self.linkMany(intervideosrc1, videoconv, video_sink)

			elif self.CAM_TYPE == "eh640":
				# gst-launch-1.0 rtpbin name=rtpbin rtspsrc location=rtsp://admin:heifu@192.168.42.108:554/live ! rtph264depay ! decodebin ! nvvidconv ! ximagesink
				intervideosrc1 = Gst.ElementFactory.make ("intervideosrc")
				intervideosrc1.set_property("channel", "t1")
				videoconv = Gst.ElementFactory.make('videoconvert')
				video_sink = Gst.ElementFactory.make("autovideosink")

				self.addMany(self.pipe_t1, intervideosrc1, videoconv, video_sink)
				self.linkMany(intervideosrc1, videoconv, video_sink)

		self.pipe_t1.set_state(Gst.State.PAUSED)

		# Save pipeline graph
		if configs.SAVE_PIPELINES == True:
			self.savePipeline(self.pipe_t1, "pipe_t1")


	# Janus streaming pipeline:
	def begin_pipe_t2(self, JANUS_IP, port):

		#"gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! nvvidconv flip-method=0 ! \
		#    nvv4l2h264enc bitrate=2000000 insert-sps-pps=true ! rtph264pay mtu=1300 ! multiudpsink clients="+configs.JANUS_IP+':'+str(msg)+" sync=false async=false"

		# nvv4l2h265enc control-rate=1 bitrate=8000000 ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse

		#gst-launch-1.0 rtspsrc location='rtsp://admin:heifu@192.168.42.108:554/cam/realmonitor?channel=1&subtype=1' latency=200 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! nvv4l2h264enc bitrate=2000000 insert-sps-pps=true ! rtph264pay mtu=1300 ! multiudpsink clients='213.63.130.233:50191'


		self.JANUS_IP = JANUS_IP
		self.port = port

		self.pipe_t2 = Gst.Pipeline.new("online-stream-pipeline")

		# Const
		encoder = 'V4L2'
		janus_resolution = (1920, 1080)#(1280, 720)

		if configs.JETSON: 
			if self.CAM_TYPE == 'default':
				intervideosrc2 = Gst.ElementFactory.make ("intervideosrc")
				intervideosrc2.set_property("channel", "t2")

				t2_videoconv_gpu_0 = Gst.ElementFactory.make('nvvidconv')
				filterCaps = Gst.ElementFactory.make('capsfilter')
				caps = Gst.Caps.from_string('video/x-raw(memory:NVMM), width=(int)' + str(janus_resolution[0]) + ', height=(int)' + str(janus_resolution[1]))
				filterCaps.set_property('caps', caps)     

				if encoder == 'V4L2':
					t2_h264enc = Gst.ElementFactory.make("nvv4l2h264enc")
					#t2_h264enc.set_property("maxperf-enabl", 1)
				elif encoder == 'OMX':
					t2_h264enc = Gst.ElementFactory.make("omxh264enc")
				t2_h264enc.set_property("bitrate", 2000000)
				#t2_h264enc.set_property("insert-sps-pps", True)
				t2_h264parse = Gst.ElementFactory.make("h264parse")
				t2_payloader = Gst.ElementFactory.make('rtph264pay')
				t2_payloader.set_property("mtu", 1500)
				t2_sink = Gst.ElementFactory.make('multiudpsink')
				clients = str(self.JANUS_IP)+":"+str(self.port)
				t2_sink.set_property("clients", clients)
				t2_sink.set_property("sync", False)
				t2_sink.set_property("async", False)

				self.addMany(self.pipe_t2, intervideosrc2, t2_videoconv_gpu_0, filterCaps, t2_h264enc, t2_h264parse, t2_payloader, t2_sink)
				self.linkMany(intervideosrc2, t2_videoconv_gpu_0, filterCaps, t2_h264enc, t2_h264parse, t2_payloader)

			elif self.CAM_TYPE == 'eh640':
				intervideosrc2 = Gst.ElementFactory.make ("intervideosrc")
				intervideosrc2.set_property("channel", "t2")

				videoconvert0 = Gst.ElementFactory.make('videoconvert')
				t2_videoconv_gpu_0 = Gst.ElementFactory.make('nvvidconv')
				
				filterCaps = Gst.ElementFactory.make('capsfilter')
				caps = Gst.Caps.from_string('video/x-raw(memory:NVMM), width=(int)' + str(janus_resolution[0]) + ', height=(int)' + str(janus_resolution[1]))
				filterCaps.set_property('caps', caps)
				t2_videoconv_gpu_1 = Gst.ElementFactory.make('nvvidconv')
				

				t2_h264enc = Gst.ElementFactory.make("nvv4l2h264enc")
				#t2_h264enc.set_property("maxperf-enabl", 1)
				t2_h264enc.set_property("bitrate", 2000000)
				t2_h264enc.set_property("insert-sps-pps", True)
				#t2_h264parse = Gst.ElementFactory.make("h264parse")
				t2_payloader = Gst.ElementFactory.make('rtph264pay')
				t2_payloader.set_property("mtu", 1300)
				t2_sink = Gst.ElementFactory.make('multiudpsink')
				clients = str(self.JANUS_IP)+":"+str(self.port)
				print("clients", clients)
				t2_sink.set_property("clients", clients)
				t2_sink.set_property("sync", False)
				t2_sink.set_property("async", False)

				#test_sink = Gst.ElementFactory.make("autovideosink")

				#filterCaps, t2_videoconv_gpu_1,
				self.addMany(self.pipe_t2, intervideosrc2, videoconvert0, t2_videoconv_gpu_0, filterCaps, t2_videoconv_gpu_1, t2_h264enc, t2_payloader, t2_sink)
				self.linkMany(intervideosrc2, videoconvert0, t2_videoconv_gpu_0, filterCaps, t2_videoconv_gpu_1, t2_h264enc, t2_payloader, t2_sink)
				
				#self.addMany(self.pipe_t2, intervideosrc2, videoconvert0, test_sink)
				#self.linkMany(intervideosrc2, videoconvert0, test_sink)


		else: # not Jetson
			intervideosrc2 = Gst.ElementFactory.make ("intervideosrc")
			intervideosrc2.set_property("channel", "t2")

			videoconvert0 = Gst.ElementFactory.make('videoconvert')
			
			filterCaps = Gst.ElementFactory.make('capsfilter')
			caps = Gst.Caps.from_string('video/x-raw,format=RGB')
			filterCaps.set_property('caps', caps)

			videoconvert1 = Gst.ElementFactory.make('videoconvert')
			t2_h264enc = Gst.ElementFactory.make('x264enc')
			t2_h264enc.set_property("pass", "qual")
			t2_h264enc.set_property("quantizer", 28)
			t2_h264enc.set_property("bitrate", 2000000)
			t2_h264enc.set_property("tune", "zerolatency")   

			filterCaps1 = Gst.ElementFactory.make('capsfilter')
			caps = Gst.Caps.from_string('video/x-h264,profile=baseline')
			filterCaps1.set_property('caps', caps) 

			#t2_h264parse = Gst.ElementFactory.make("h264parse")
			t2_payloader = Gst.ElementFactory.make('rtph264pay')
			t2_payloader.set_property("mtu", 1300)
			t2_sink = Gst.ElementFactory.make('multiudpsink')
			clients = str(self.JANUS_IP)+":"+str(self.port)
			t2_sink.set_property("clients", clients)
			t2_sink.set_property("sync", False)
			t2_sink.set_property("async", False)

			self.addMany(self.pipe_t2, intervideosrc2, videoconvert0, filterCaps, videoconvert1, t2_h264enc, filterCaps1, t2_payloader, t2_sink)

			self.linkMany(intervideosrc2, videoconvert0, filterCaps, videoconvert1, t2_h264enc, filterCaps1, t2_payloader, t2_sink)

		self.pipe_t2.set_state(Gst.State.PAUSED)
		print("GST_PIPES: pipe_t2 ready")

		# Save pipeline graph
		if configs.SAVE_PIPELINES == True:
			self.savePipeline(self.pipe_t2, "pipe_t2")


	# recording pipeline:
	def begin_pipe_t3(self, rec_path, curr_datetime): 

		rec_resolution = (1920, 1080)#(3840, 2160) #(1920, 1080)

		# v1
		#nvv4l2h265enc control-rate=1 bitrate=8000000 ! 'video/x-h265, stream-format=(string)byte-stream' ! h265parse ! qtmux ! filesink location=test.mp4 -e

		# v2
		#gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=4032, height=3040, framerate=30/1,  format=(string)NV12' ! nvvidconv top=440 bottom=2600 left=96 right=3936 ! 'video/x-raw(memory:NVMM), width=3840, height=2160,  format=(string)NV12' ! nvv4l2h264enc bitrate=50000000 ! h264parse ! qtmux ! filesink location=4k_h264_.mp4

		if self.TeePipe.get_state(1)[1] != Gst.State.PLAYING:
			#print(f"TeePipe state: {self.TeePipe.get_state(Gst.CLOCK_TIME_NONE)}")
			print("!! ERROR-GST_PIPES: Main pipeline is not currently playing, exiting sub-pipeline..")
			return

		self.pipe_t3 = Gst.Pipeline.new("test-pipeline")

		if configs.JETSON: 
			intervideosrc3 = Gst.ElementFactory.make ("intervideosrc")
			intervideosrc3.set_property("channel", "t3")

			t3_videoconv_gpu_0 = Gst.ElementFactory.make('nvvidconv')
			filterCaps = Gst.ElementFactory.make('capsfilter')
			caps = Gst.Caps.from_string('video/x-raw(memory:NVMM), width=(int)' + str(rec_resolution[0]) + ', height=(int)' + str(rec_resolution[1]) + ', format=NV12')
			filterCaps.set_property('caps', caps)

			h265enc = Gst.ElementFactory.make("nvv4l2h265enc")

			h265enc.set_property("control-rate", 1)
			h265enc.set_property("bitrate", 20000000) # 8000000
			h265enc.set_property("insert-sps-pps", 1)

			h265parse = Gst.ElementFactory.make("h265parse")
			qtmux = Gst.ElementFactory.make("qtmux")
			filesink = Gst.ElementFactory.make("filesink")
			filename = rec_path+"rec--"+curr_datetime+".mp4"
			filesink.set_property("location", filename)

			# Pipeline for t3
			self.pipe_t3.add(intervideosrc3)
			self.pipe_t3.add(t3_videoconv_gpu_0)
			self.pipe_t3.add(filterCaps)
			self.pipe_t3.add(h265enc)
			self.pipe_t3.add(h265parse)
			self.pipe_t3.add(qtmux)
			self.pipe_t3.add(filesink)

			# Linking
			link_ret = [None] * 10
			link_ret[0] = intervideosrc3.link(t3_videoconv_gpu_0)
			link_ret[1] = t3_videoconv_gpu_0.link(filterCaps)
			link_ret[2] = filterCaps.link(h265enc)
			link_ret[3] = h265enc.link(h265parse)
			link_ret[4] = h265parse.link(qtmux)
			link_ret[5] = qtmux.link(filesink)
			print("> Linking pipe_t3 elements..")
			for i in range(len(link_ret)):
				if link_ret[i] not in {None, True}:
					print("GST_PIPES: Link "+str(i)+" status (True=OK): "+str(link_ret[i]))

		else: # not Jetson
			intervideosrc3 = Gst.ElementFactory.make ("intervideosrc")
			intervideosrc3.set_property("channel", "t3")
			videoconvert = Gst.ElementFactory.make('videoconvert')

			h264enc = Gst.ElementFactory.make("x264enc")
			h264enc.set_property("bitrate", 2000000)

			h264parse = Gst.ElementFactory.make("h264parse")
			qtmux = Gst.ElementFactory.make("qtmux")
			filesink = Gst.ElementFactory.make("filesink")
			filename = rec_path+"rec--"+curr_datetime+".mp4"
			filesink.set_property("location", filename)

			self.addMany(self.pipe_t3, intervideosrc3, videoconvert, h264enc, h264parse, qtmux, filesink)

			self.linkMany(intervideosrc3, videoconvert, h264enc, h264parse, qtmux, filesink)

		# Start with pipeline paused
		self.pipe_t3.set_state(Gst.State.PAUSED)
		print("GST_PIPES: pipe_t3 ready and waiting")

		# Save pipeline graph
		if configs.SAVE_PIPELINES == True:
			self.savePipeline(self.pipe_t3, "pipe_t3")


	# Snapshot pipeline
	def begin_pipe_t4(self, snap_path, curr_datetime):

		if self.TeePipe.get_state(1)[1] != Gst.State.PLAYING:
			#print(f"TeePipe state: {self.TeePipe.get_state(Gst.CLOCK_TIME_NONE)}")
			print("!! ERROR-GST_PIPES: Main pipeline is not currently playing, exiting sub-pipeline..")
			return

		# gst-launch-1.0 v4l2src ! jpegenc snapshot=1 ! filesink location=snap.jpg

		photo_resolution = (3840, 2160)#(1920, 1080)

		self.pipe_t4 = Gst.Pipeline.new("snap-pipeline")

		if configs.JETSON:        
			
			intervideosrc4 = Gst.ElementFactory.make ("intervideosrc")
			intervideosrc4.set_property("channel", "t4")

			t4_videoconv_gpu_0 = Gst.ElementFactory.make('nvvidconv')
			filterCaps = Gst.ElementFactory.make('capsfilter')
			caps = Gst.Caps.from_string('video/x-raw(memory:NVMM), width=(int)' + str(photo_resolution[0]) + ', height=(int)' + str(photo_resolution[1]))
			filterCaps.set_property('caps', caps)

			videoconv_gpu = Gst.ElementFactory.make('nvvidconv')
			videoconv = Gst.ElementFactory.make('videoconvert')

			jpegenc = Gst.ElementFactory.make('jpegenc')
			jpegenc.set_property('snapshot', 1)

			filesink = Gst.ElementFactory.make("filesink")
			filename = snap_path+"snap--"+curr_datetime+".jpg"
			filesink.set_property("location", filename)

			self.addMany(self.pipe_t4, intervideosrc4, t4_videoconv_gpu_0, filterCaps, videoconv_gpu, videoconv, jpegenc, filesink)

			# Linking
			link_ret = [None] * 10
			link_ret[0] = intervideosrc4.link(t4_videoconv_gpu_0)
			link_ret[1] = t4_videoconv_gpu_0.link(filterCaps)
			link_ret[2] = filterCaps.link(videoconv_gpu)
			link_ret[3] = videoconv_gpu.link(videoconv)
			#self.print_pad_capabilities(videoconv, "src")
			link_ret[4] = videoconv.link(jpegenc)
			link_ret[5] = jpegenc.link(filesink)
			print("Linking pipe_t2 elements..")
			for i in range(len(link_ret)):
				if link_ret[i] not in {None, True}:
					print("Link "+str(i)+" status (True=OK): "+str(link_ret[i]))

			# Start with pipeline paused
			self.pipe_t4.set_state(Gst.State.PLAYING)
			print("GST_PIPES: pipe_t4: photo saved as "+filename)

		else: # not JETSON

			intervideosrc4 = Gst.ElementFactory.make ("intervideosrc")
			intervideosrc4.set_property("channel", "t4")
			videoconv = Gst.ElementFactory.make('videoconvert')
			jpegenc = Gst.ElementFactory.make('jpegenc')
			jpegenc.set_property('snapshot', 1)
			filename = snap_path+"snap--"+curr_datetime+".jpg"
			filesink = Gst.ElementFactory.make("filesink")
			filesink.set_property("location", filename)

			self.addMany(self.pipe_t4, intervideosrc4, videoconv, jpegenc, filesink)

			#print("GST_PIPES: Linking pipe_t2 elements..")
			self.linkMany(intervideosrc4, videoconv, jpegenc, filesink)

			# Start pipeline which immediately closes after photo is taken
			self.pipe_t4.set_state(Gst.State.PLAYING) 
			print("GST_PIPES: photo saved as "+filename)

		# Save pipeline graph
		if configs.SAVE_PIPELINES == True:
			self.savePipeline(self.pipe_t4, "pipe_t4")
		

	def startRecord(self):
		if self.recording_state == "starting" or self.recording_state == "waiting":
			curr_datetime = datetime.now()
			curr_datetime_str = curr_datetime.strftime("%d-%m-%Y--%H-%M-%S")

			self.begin_pipe_t3(configs.REC_PATH, curr_datetime_str)
			#self.begin_pipe_t4(configs.PHOTO_PATH, curr_datetime_str)

			#get time and change output file name
			self.pipe_t3.set_state(Gst.State.PLAYING)
			self.recording_state = "recording"
			print("GST_PIPES: Recording started at "+curr_datetime_str)

	def stopRecord(self):
		if self.recording_state == "recording":
			Gst.Element.send_event(self.pipe_t3, Gst.Event.new_eos())
			#self.pipe_t3.set_state(Gst.State.PAUSED)
			self.recording_state = "waiting"
			print("GST_PIPES: Recording stopped")


	def takePhoto(self):
		curr_datetime = datetime.now()
		curr_datetime_str = curr_datetime.strftime("%d-%m-%Y--%H-%M-%S")
		self.begin_pipe_t4(configs.PHOTO_PATH, curr_datetime_str)

	# "thermal" bool - 0: main; 1: thermal
	def switchThermal(self, enable_thermal):

		#
		if self.port is None or self.JANUS_IP is None:
			print("Port not defined yet")
			return

		# Stop current stream
		self.TeePipe.set_state(Gst.State.NULL)
		print("GST_PIPES: TeePipe stopped")
		self.pipe_t2.set_state(Gst.State.NULL)
		print("GST_PIPES: pipe_t2 stopped")

		# Build/Start new Tee pipe with different stream
		self._buildPipeline(thermal=enable_thermal)
		self.startTeePipe()

		# Build/start pipe_t2 stream to janus IP and port
		self.startOnlineStream(self.JANUS_IP, self.port)
		

	def print_pad_capabilities(self, element, pad_name):
		# retrieve pad
		pad = element.get_static_pad(pad_name)
		if not pad:
			print("Could not retrieve pad.")
			sys.exit(1)
		
		# Retrieve negotiated caps (or acceptable caps if negotiation is not finished yet)
		caps = pad.get_current_caps()
		if not caps:
			caps = pad.query_caps()

		print("Caps for the %s pad:" % pad_name)
		print(caps.to_string()) # print_caps?
		#pad.unref()

	




