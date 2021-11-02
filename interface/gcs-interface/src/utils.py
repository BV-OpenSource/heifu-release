#!/usr/bin/env python

class File:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbFilename(self, msg):
        self.vehicle.logger.info(msg)
        self.vehicle.socket.emit('/filename', msg.data)