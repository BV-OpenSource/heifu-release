#!/usr/bin/env python

class StatusReport:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbStatus(self, msg):
        self.vehicle.socket.emit('/c/s', ({'m': msg.data}))

    def cbFCUErr(self, msg):
        self.vehicle.socket.emit('/m/fcu/e', msg.data)

    def cbAcclStatus(self, msg):
        self.vehicle.logger('%s' % str(msg))
        self.vehicle.socket.emit('/m/a/s', msg.data)

    def cbAPMInit(self, msg):
        self.vehicle.socket.emit('/m/apm/s', msg.data)


class MagReport:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbMagStatus(self, msg):
        self.vehicle.logger('Mag message-> %s' % str(msg))
        self.vehicle.socket.emit('/m/m/s', msg.data)

    def cbMagReport(self, msg):
        self.vehicle.logger('Mag reporter: %s' % str(msg))
        self.vehicle.socket.emit(
            '/m/m/r', {'r': msg.report, 'c': msg.confidence})