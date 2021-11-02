#!/usr/bin/env python

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import std_msgs.msg

class Takeoff:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbTakeOff(self, msg):
        self.vehicle.socket.emit('/takeoff', {'msg': True})

    def cbTakeOffDiagnostic(self, msg):
        self.vehicle.logger.info('Send takeoff ACK-> %s' % str(msg.data))
        self.vehicle.socket.emit('/mission/takeoffACK', msg.data)
        self.vehicle.socket.emit('/takeoff', msg.data)

    def onTakeOff(self, msg):
        print('Takeoff received')
        # global on_mission
        if(self.vehicle.uav_position.latitude is not None and self.vehicle.uav_position.longitude is not None):
            point = Point(self.vehicle.uav_position.latitude,
                          self.vehicle.uav_position.longitude)
            polygons = self.vehicle._nfz.calculatePolygon(True)
            if polygons:
                for polygon in polygons:
                    if polygon.contains(point) == True:
                        self.vehicle.logger.info(
                            "I can't takeoff. I'm inside a No-Fly Zone.")
                        return
        takeoffAlt = std_msgs.msg.UInt8()
        takeoffAlt.data = 10
        self.vehicle.pubTakeoff.publish(takeoffAlt)
        self.vehicle.logger.info('TAKEOFF')
        self.vehicle._state.onMission = False

class Land:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbLand(self, msg):
        self.vehicle.socket.emit('/land', {'msg': True})

    def cbLandDiagnostic(self, msg):
        self.vehicle.logger.info('Send land ACK-> %s' % str(msg.data))
        self.vehicle.socket.emit('/mission/landACK', msg.data)  # IS IT NEEDED?
        self.vehicle.socket.emit('/land', msg.data)

    def onLand(self, msg):
        self.vehicle.pubLand.publish()
        self.vehicle.logger.info('LAND')
        self.vehicle._state.onMission = False

class Arm:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbArm(self, msg):
        self.vehicle.socket.emit('/arm', '')

    def cbDisarm(self, msg):
        self.vehicle.socket.emit('/disarm', '')

class RTL:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def onRlt(self, msg):
        self.vehicle.pubRtl.publish()
        self.vehicle.logger.info('RTL')
        self.vehicle._state.onMission = False

class Command:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def onCommand(self, msg):
        self.vehicle.logger.info('Received command: ')
        self.vehicle.logger.info('%s' % str(msg['command']))
        self.vehicle.srvCmd(msg['broadcast'], msg['command'], msg['confirmation'], msg['param1'], msg['param2'], msg['param3'],
                    msg['param4'], msg['param5'], msg['param6'], msg['param7'])

    def onCommandACK(self, msg):
        self.vehicle.logger.info('Received command ACK')
        self.vehicle.srvCmdAck(msg['command'], msg['result'],
                       msg['progress'], msg['result_param2'])
