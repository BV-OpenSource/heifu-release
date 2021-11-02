#!/usr/bin/env python

import std_msgs.msg

class GSTreamer:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.isRecording = False
        self.isStreaming = False
        self.port = -1

    def onPortReceived(self, port):
        self.port = port

    def onStartTStream(self, port=-1):

        # leave an option to send the port on the onStartTStream method
        # required to validate if the self.port has already been set
        if self.port != port:
            self.port = port

        if self.isStreaming:
            self.vehicle.logger.info('Already streaming. Ignoring start stream request.')
            return

        self.vehicle.pubStartTStream.publish()
        #self.vehicle.pubStartTStream.publish(port)
        self.vehicle.logger.info('Start stream')
        self.isStreaming = True


    def onStopTStream(self):
        if not self.isStreaming:
            self.vehicle.logger.info('No stream running. Ignoring stop stream request.')
            return

        self.vehicle.pubStopTStream.publish()
        self.vehicle.logger.info('Stop Stream')
        self.isStreaming = False


    def onStartRecording(self):
        if self.isRecording:
            self.vehicle.logger.info('Already recording. Ignoring start recording request.')
            return

        self.vehicle.pubStartRecording.publish()
        self.vehicle.logger.info('Start recording')
        self.isRecording = True

    def onStopRecording(self):
        if not self.isRecording:
            self.vehicle.logger.info('No recording running. Ignoring stop recording request.')
            return

        self.vehicle.pubStopRecording.publish()
        self.isRecording = False


    def onTakePhoto(self):
        self.vehicle.pubTakePhoto.publish()
        self.vehicle.logger.info('Take Photo')
