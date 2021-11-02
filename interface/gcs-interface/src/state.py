#!/usr/bin/env python
import pickle
import os

PROJECTPATH = os.path.dirname(os.path.abspath(__file__))

class Pose:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbPose(self, msg):
        self.vehicle.socket.emit('/m/lp/o', {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y,
                                             'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w})
        self.vehicle.socket.emit('/m/lp/a', ({'a': msg.pose.position.z}))


class Velocity:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbVel(self, msg):
        self.vehicle.socket.emit(
            '/m/lp/v', ({'x': msg.twist.linear.x, 'y': msg.twist.linear.y, 'z': msg.twist.linear.z}))


class GPS:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbGPS(self, msg):
        if(msg.altitude and msg.latitude and msg.longitude):
            self.vehicle.uav_position.latitude = msg.latitude
            self.vehicle.uav_position.longitude = msg.longitude
            self.vehicle.uav_position.altitude = msg.altitude

            # self.vehicle._nfz.calculateAzimuthAndDistances(self.vehicle.uav_position)
            if (not self.vehicle._nfz.nfz_complete):
                self.vehicle._nfz.setupNFZ()
            self.vehicle._nfz.calculateAzimuthAndDistances(
                self.vehicle.uav_position)
            self.vehicle.socket.emit(
                '/m/gp/g', {'a': msg.altitude, 'x': msg.latitude, 'y': msg.longitude})

    def cbGpsFix(self, msg):
        self.vehicle.socket.emit('/g/f/m', msg.data)

    def cbSatNum(self, msg):
        self.vehicle.socket.emit('/s', ({'n': msg.data}))


class State:
    def __init__(self, vehicle):
        self.vehicle = vehicle

        self.mode = ""
        self.onManual = True
        self.onMission = False
        self.isLanded = False
        self.isArmed = False

    def cbState(self, msg):
        self.mode = msg.mode

        # FROM https://mavlink.io/en/messages/common.html#MAV_STATE
        self.isLanded = (msg.system_status < 4) or (msg.system_status > 6) 
    
        self.onManual = (msg.mode != "AUTO" and msg.mode != "GUIDED")

        self.onMission = (msg.armed and msg.mode == "AUTO")

        self.isArmed = msg.armed

#        if(msg.mode != "AUTO" and msg.mode != "GUIDED"):
#            self.onManual = True
#        else:
#            self.onManual = False
#        if (msg.armed and msg.mode == "AUTO"):
#            self.onMission = True
#        else:
#            self.onMission = False
        self.vehicle.socket.emit('/m/s', ({'connected': msg.connected, 'armed': msg.armed,
                                           'guided': msg.guided, 'manual_input': msg.manual_input, 'mode': msg.mode,
                                           'system_status': msg.system_status}))


class Battery:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        if not self.vehicle.simulation:
            with open( os.path.join(PROJECTPATH, 'battery.pkl'), 'rb') as file:
                    self.battery_percentage = pickle.load(file)

    def cbBattery(self, msg):
        if not self.vehicle.simulation:
            percentage = self.battery_percentage(msg.voltage).item()
            # Clamp value from 0 to 100
            percentage = max(0, min(100, percentage))
            percentage = percentage * 100
        else:
            percentage = msg.percentage * 100
        self.vehicle.socket.emit('/b', {'p': percentage})
