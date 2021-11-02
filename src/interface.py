#!/usr/bin/env python

from os import path, system, remove, mkdir
from base64 import b64decode, b64encode
from Crypto.Cipher import AES
import hashlib
import requests
import socketio
import argparse
import time

import sys
from os import path
from config import configs

uav_interface = __import__(configs.MODULE_NAME)

print("ARGS", sys.argv)

# Argument parser
parser = argparse.ArgumentParser(description='Arguments for GCS interface.')
parser.add_argument("-n", "--name",     type=str,   default='heifu0',            help='Namespace')
parser.add_argument("-e", "--endpoint", type=str,   default='preprod',           help='Endpoint')
parser.add_argument("-c", "--camtype",  type=str,   default='default',           help='Camera type')
parser.add_argument("-co", "--configname",  type=str,   default='config',           help='Config name')
parser.add_argument(      "--jetson",   action="store_true", help='Working on Jetson.')
parser.add_argument(      "--nfz",      action="store_true", help='Enable No-Fly Zones.')
parser.add_argument("-v", "--version",  help="show program version",             action="store_true")
args = parser.parse_known_args()[0]

if args.version:
    print("Version 0.8.")
    exit()

ROSNAME = args.name
configname = args.configname + '.bext'
configs.URL = configs.BACKEND_LIST[args.endpoint] + configs.API_VERSION
configs.URL_WS = configs.BACKEND_LIST[args.endpoint] + configs.API_VERSION
configs.JANUS_IP = configs.JANUS_LIST[args.endpoint]
configs.ENABLE_NFZ = args.nfz
configs.JETSON = args.jetson
configs.CAM_TYPE = args.camtype

class SIO:
    def __init__(self, socket, ID):
        self.sio = socket
        self.ID = ID
        self.sio.on('connect', self.connect)
        self.sio.on('disconnect', self.disconnect)
        self.sio.on(ID + '/port', self.onStream)
        self.sio.on(ID + '/janusDown', self.stopStream)
        self.sio.on(ID + '/shutdown', self.onShutdown)
        self.sio.on(ID + '/frontend/takeoff', self.takeOff)
        self.sio.on(ID + '/frontend/gimbal', self.gimbal)
        self.sio.on(ID + '/mavros/command', self.mavrosCommand)
        self.sio.on(ID + '/mavros/command_ack', self.commandACK)
        self.sio.on(ID + '/frontend/land', self.onLand)
        self.sio.on(ID + '/mission/get', self.onGetMission)
        self.sio.on(ID + '/mission/push', self.onPush)
        self.sio.on(ID + '/mission/start', self.onMissionStart)
        self.sio.on(ID + '/mission/stop', self.onMissionCancel)
        self.sio.on(ID + '/mission/clear', self.onMissionClear)
        self.sio.on(ID + '/mission/resume', self.onMissionResume)
        self.sio.on(ID + '/frontend/rtl', self.onRlt)
        self.sio.on(ID + '/frontend/setpoint', self.onSetpoint)
        self.sio.on(ID + '/logrequest', self.logRequest)
        self.sio.on(ID + '/frontend/sendTriggerSensor', self.sendTriggerRISESensor)

    def emit(self, topic, msg):
        endpoint = self.ID + topic
        self.sio.emit(endpoint, msg)

    # defines on connect event and prints if connection is established
    def connect(self):
        print('Connection established')

    # defines on disconnect event and prints if connection is lost
    def disconnect(self):
        self.ros_interface._gst.onStopTStream()
        print('Disconnected from server.')

    def onStream(self, msg):
        print('Port received:', msg)

        # Sometimes the init_node takes longer then the server to answer with the port
        # making it impossible to send the result in the respective topics
        # for this reason we have to wait for the ros_interface to be ready
        while not self.ros_interface.isInited:
            time.sleep(1) # sleep 1 sec

        # TODO - Do not start stream right away... The RC Controller should push a start stream button
        self.ros_interface._gst.onPortReceived(msg)
        self.ros_interface._gst.onStartTStream(msg)
        self.emit('/portACK', '')


    def stopStream(self, msg):
        self.ros_interface._gst.onStopTStream()
        print('Janus went down')

    def onShutdown(self, msg):
        self.sio.disconnect()
        print('Shuting down',msg)

    def takeOff(self, msg):
        self.ros_interface._takeoff.onTakeOff(msg)

    # #@sio.on(ID + '/frontend/cmd')
    # def cmd(msg):
    #     ros_interface.onCmd(msg)

    #
    def gimbal(self, msg):
        self.ros_interface._gimbal.onGimbal(msg)

    def mavrosCommand(self, msg):
        self.ros_interface._command.onCommand(msg)

    def commandACK(self, msg):
        self.ros_interface._command.onCommandACK(msg)

    def onLand(self, msg):
        self.ros_interface._land.onLand(msg)

    def onGetMission(self, msg):
        self.ros_interface._mission.onGetMission(msg)

    def onPush(self, msg):
        self.ros_interface._mission.onPush(msg)

    def onMissionStart(self, msg):
        self.ros_interface._mission.onMissionStart(msg)

    def onMissionCancel(self, msg):
        self.ros_interface._mission.onMissionCancel(msg)

    def onMissionResume(self, msg):
        self.ros_interface._mission.onMissionResume(msg)

    def onMissionClear(self, msg):
        self.ros_interface._mission.onMissionClear(msg)

    def onRlt(self, msg):
        self.ros_interface._rtl.onRlt(msg)

    def onStartTStream(self, msg):
        self.ros_interface._gst.onStartTStream(msg)

    def onStopTStream(self, msg):
        self.ros_interface._gst.onStopTStream(msg)

    def onStartRecording(self, msg):
        self.ros_interface._gst.onStartRecording(msg)

    def onStopRecording(self, msg):
        self.ros_interface._gst.onStopRecording(msg)

    def onTakePhoto(self, msg):
        self.ros_interface._gst.onTakePhoto(msg)

    def onSetpoint(self, msg):
        self.ros_interface._setpoint.onSetpoint(msg)

    def logRequest(self, msg):
        self.emit('/lr', msg)

    def sendTriggerRISESensor(self, msg):
        self.ros_interface._rfid.sendTriggerRISESensor(msg)

class vehicleInterface:
    def __init__(self):
        decripted = self.getConfig()
        if (decripted == -1):
            exit()
        assetname = decripted.split(',')[0]
        self.password = decripted.split(',')[1]
        self.ID = decripted.split(',')[2]
        token = self.getToken().decode("utf-8")

        print('Asset:: ' + assetname + ' ID::' + self.ID)
        print('With the following token: ' + token)

        sio = socketio.Client(  reconnection=True,
                                logger=False, engineio_logger=False,
                                reconnection_delay= 5,
                                reconnection_delay_max= 10,
                                reconnection_attempts=10)

        # connects to backend
        sio.connect(configs.URL_WS + '?source=drone&token=' + token)

        socketio_int = SIO(sio, self.ID)

        self._ros_interface = uav_interface.UAV(str(ROSNAME), socketio_int, configs.ENABLE_NFZ, 1-configs.JETSON)

    # Access config file in home directory
    def getConfig(self):
        try:
            home_dir = path.expanduser("~")
            config_file_path = path.abspath(path.join(home_dir, configname))
            print(config_file_path)
            file = open(config_file_path, "r")
            line = file.readline().rstrip()
            namespace = self.decrypt(line)
        except:
            print('File not found')
            time.sleep(5) # Sleep for 5 seconds
            return -1
        return namespace

    # Decrypt cipher from config file
    def decrypt(self, encryption):
        key = b64encode(hashlib.sha256(b'BeyondSkyline19').digest())[0:32]
        encryption = b64decode(encryption)
        cipher = AES.new(key, AES.MODE_ECB)
        unpad = lambda s: s[:-ord(s[len(s) - 1:])]
        decripted = unpad(cipher.decrypt(encryption)).decode('utf8').split('\n')[1]
        return decripted

    # Get token after login
    def getToken(self):
        url = configs.URL + '/asset/auth'

        payload = 'assetId=' + self.ID + '&password=' + self.password
        headers = {
            'Content-Type': 'application/x-www-form-urlencoded'
        }

        response = requests.request("POST", url, headers=headers, data=payload)
        token = response.text.encode('utf8')
        return token

    def main(self):
        self._ros_interface.start()

if __name__ == '__main__':
    interface = vehicleInterface()
    interface.main()
