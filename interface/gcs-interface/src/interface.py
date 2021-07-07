#!/usr/bin/env python

from os import path, system, remove, mkdir
from base64 import b64decode, b64encode
from Crypto.Cipher import AES
import hashlib
import requests
import socketio
import time
import subprocess

import sys
from os import path
from config import configs

# module_name = 'heifu_interface'
# ros_interface = __import__(module_name)
#import heifu_interface
uav_interface = __import__(configs.MODULE_NAME)

print("ARGS", sys.argv)

if (len(sys.argv)>3): ## roslaunch adds 2 arguments
    ROSNAME = sys.argv[1]
    configname = '-' + ROSNAME
    configs.URL = configs.BACKEND_LIST[sys.argv[2]] + configs.API_VERSION
    configs.URL_WS = configs.BACKEND_LIST[sys.argv[2]] + configs.API_VERSION
    configs.JANUS_IP = configs.JANUS_LIST[sys.argv[2]]
else:
    ROSNAME = 'heifu0'
    configname = ''

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
        self.sio.on(ID + '/mission/resume', self.onMissionResume)
        self.sio.on(ID + '/frontend/rtl', self.onRlt)
        self.sio.on(ID + '/frontend/setpoint', self.onSetpoint)
        self.sio.on(ID + '/logrequest', self.logRequest)
        self.sio.on(ID + '/frontend/sendTriggerSensor', self.sendTriggerRISESensor)
        self.stream_process =  None;

    def emit(self, topic, msg):
        endpoint = self.ID + topic
        self.sio.emit(endpoint, msg)

    # defines on connect event and prints if connection is established
    def connect(self):
        print('connection established')

    # defines on disconnect event and prints if connection is lost
    def disconnect(self):
        if self.stream_process is not None:
            self.stream_process.terminate()
            self.stream_process.wait()

        print('Disconnected from server.')

    def onStream(self, msg):
        print('Port received:', msg)

        startStream = self.selectStream(msg)
        self.stream_process = subprocess.Popen(['bash', '-c', startStream], shell=False)  # Run the process to video recorder
        self.emit('/portACK', '')
    
    ###################################################   Stream 
    def selectStream(self, msg):

        ####### Legion Camera for testing is the default
        stream_cmd = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc quantizer=25 \
            tune=zerolatency bitrate=4096 ! \"video/x-h264,profile=baseline\"  ! rtph264pay ! multiudpsink clients=127.0.0.1:" + str(
            msg) + ',' + configs.JANUS_IP + ':' + \
            str(msg) + " sync=false async=false"

        if configs.STABILIZE:
            if configs.JETSON:
                stab_string = "stabilization_jetson"
            else:
                stab_string = "stabilization_pc"
            if path.exists("./" + stab_string) == False:
                print("Stabilization executable not found, disabling.")
                configs.STABILIZE = False
        if configs.STABILIZE:
            record_str = ' -r' if record else ''
            stream_cmd = "./" + stab_string + " -p " + str(msg) + " -d 0 -a " + str(configs.JANUS_IP) + " -w 640 -e 480 -f 30 -c laptop" + record_str
            if configs.JETSON:
                switcher = {
                    "realsense": "./" + stab_string + " -p " + str(msg) + " -d " + str(configs.DEV_ID) + " -a " + str(configs.JANUS_IP) + " -w 1280 -e 720 -f 30 -c realsense -j" + record_str,
                    "raspy": "./" + stab_string + " -p " + str(msg) + " -d " + str(configs.DEV_ID) + " -a " + str(configs.JANUS_IP) + " -w 1920 -e 1080 -f 30 -c raspy -j" + record_str,
                }
            else:
                switcher = {
                    "realsense": "./" + stab_string + " -p " + str(msg) + " -d " + str(configs.DEV_ID) + " -a " + str(configs.JANUS_IP) + " -w 1280 -e 720 -f 30 -c realsense" + record_str,
                }
        else:
            ####### Legion Camera for testing is the default
            stream_cmd = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,framerate=10/1,width=1280,height=720 ! videoconvert ! x264enc quantizer=25 \
                tune=zerolatency bitrate=4096 ! \"video/x-h264,profile=baseline\"  ! rtph264pay mtu=1300 ! multiudpsink clients=127.0.0.1:" + str(
                msg) + ',' + configs.JANUS_IP + ':' + \
                        str(msg) + " sync=false async=false"
            if configs.JETSON:
                switcher = {
                    "realsense": "gst-launch-1.0 -v v4l2src device=/dev/video" + str(configs.DEV_ID) + " ! 'video/x-raw,format=(string)YUY2, width=1280, height=720, framerate=30/1' ! nvvidconv !  \
                    nvv4l2h264enc control-rate=0 insert-sps-pps=1 bitrate=2048000 profile=0 ! rtph264pay mtu=1300 ! multiudpsink \
                    clients=" + configs.JANUS_IP + ':' + str(msg) + ", sync=false async=false",
                    "raspy": "gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! nvvidconv flip-method=2 ! \
                    nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1300 ! multiudpsink clients=" + configs.JANUS_IP + ':' + str(
                        msg) + \
                            " sync=false async=false",
                    "insta360": "gst-launch-1.0 -v rtspsrc location=rtsp://" + str(configs.INSTA360_IP) + "/live/live latency=0 ! multiudpsink \
                    clients=" + configs.JANUS_IP + ':' + str(msg) + " sync=false ",
                }
            else:
                switcher = {
                    "realsense": "gst-launch-1.0 -v v4l2src device=/dev/video" + str(configs.DEV_ID) + " ! 'video/x-raw,format=(string)YUY2, width=1280, height=720, framerate=30/1' ! videoconvert !  \
                    x264enc pass=qual quantizer=28 bitrate=8192 tune=zerolatency  ! \"video/x-h264,profile=baseline\" ! rtph264pay mtu=1300 ! multiudpsink \
                    clients=" + configs.JANUS_IP + ':' + str(msg) + ", sync=false async=false",
                    "insta360": "gst-launch-1.0 -v rtspsrc location=rtsp://" + str(configs.INSTA360_IP) + "/live/live latency=0 ! multiudpsink \
                    clients=" + configs.JANUS_IP + ':' + str(msg) + " sync=false ",
                    "gazebo": "gst-launch-1.0 -v v4l2src device=/dev/video9 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8192 \
                        ! \"video/x-h264,profile=baseline\" ! rtph264pay mtu=1300 ! multiudpsink clients=127.0.0.1:5000," + configs.JANUS_IP + ":" + str(msg) + " sync=false async=false",
                    "gazebo_integrated": "gst-launch-1.0  -v udpsrc port=5700 \
                                         ! multiudpsink clients=127.0.0.1:5000," + configs.JANUS_IP + ":" + str(msg) + "sync=false"
                }
        # Remove uppercase and spaces        
        cam_type = configs.CAM_TYPE.replace(' ', '').lower()
        return switcher.get(cam_type, stream_cmd)

    def stopStream(self, msg):
        if self.stream_process is not None:
            print('Janus went down')
            self.stream_process.terminate()
            self.stream_process.wait()

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

    def onRlt(self, msg):
        self.ros_interface._rtl.onRlt(msg)

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
            dir = path.abspath(path.join(home_dir, 'config' + configname))
            file = open(dir, "r")
            line = file.readline().rstrip()
            namespace = self.decrypt(line)
        except:
            print('File not found')
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
