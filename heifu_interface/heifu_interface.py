#!/usr/bin/env python
import rospy
import std_msgs
from std_msgs.msg import Bool
import geometry_msgs.msg
import sensor_msgs.msg
import geographic_msgs.msg
import mavros_msgs
import socketio
import json
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, CommandLong, CommandBool
from mavros_msgs.msg import WaypointList, Waypoint, WaypointReached, MagnetometerReporter
import os
from base64 import b64decode
from base64 import b64encode
from Crypto.Cipher import AES
import hashlib
import numpy as np
from os.path import expanduser
import requests
import subprocess
import heifu_definitions
import glob
import requests
from gimbal.srv import setGimbalAxes, setGimbalAxesRequest
import os.path
from os import path
from os import listdir
import time
import log

logger = log.HEIFULogger(os.path.basename(__file__), writeFile=True)

HEIFU_INFO = logger.info
HEIFU_WARNING = logger.warning
HEIFU_ERROR = logger.error
HEIFU_EXCEPTION = logger.exception
HEIFU_DEBUG = logger.debug

# Select the endpoint target: preprod, qa, local, or graca
endpoint = 'local'

# Cameras available: PC: default, RealSense: realsense, RaspyCam: raspy, Insta360 Pro: insta360
camType = "insta360"

# To enable jetson streaming pipelines set to True
jetsonON = False
# TODO Use gstreamer monitor to discover video capture devices and their paths
devId = 2  # Id for realsense

# Preprod
# URL = 'https://beyond-skyline-backend.preprod.pdmfc.com'
# URLWS = 'https://beyond-skyline-backend.preprod.pdmfc.com'
# QA
# URL = 'https://beyond-skyline-backend.qa.pdmfc.com'
# URLWS = 'https://beyond-skyline-backend.qa.pdmfc.com'
# Graca
# URL = 'http://85.246.61.135:3000'
# URLWS = 'http://85.246.61.135:3000'

# Local
# URL = 'http://127.0.0.1:3000'
# URLWS = 'http://127.0.0.1:3000'

backendList = {'preprod': 'https://beyond-skyline-backend.preprod.pdmfc.com',
               'qa': 'https://beyond-skyline-backend.qa.pdmfc.com', \
               'local': 'http://127.0.0.1:3000', 'graca': 'http://85.246.61.135:3000'}
janusList = {'preprod': 'janus.preprod.pdmfc.com', 'qa': 'janus.qa.pdmfc.com', 'local': '127.0.0.1',
             'graca': '85.246.61.135'}

URL = backendList[endpoint]
URLWS = backendList[endpoint]
janusIP = janusList[endpoint]

insta360IP = '192.168.1.162'
# ip graca: 85.243.249.239
UPLOADURL = URL + '/drone/uploadVideo'
DOWNLOADAUDIOURL = URL + '/drone/downloadAudio'
VIDEOPATH = '/home/heifu/Videos/'
VIDEOLIST = '/home/heifu/Videos/*.mp4'
PROJECTPATH = '/home/heifu/heifu_ws/src/heifu/heifu_interface/'

# Remove uppercase and spaces
camType = camType.replace(' ', '').lower()

global pidProcess
global TOKEN
global sendindVideo
global isMissionStarted
global totalWaypointsMission
global lastBatteryValue
global waypointsCode
global waypointCounter
# Initiated is used in order to only subscribe to Rostopics Once
global initiated
initiated = False

minimumVoltage = 20.4
maximumVoltage = 25.2
slope = (100.0 - 0.0) / (maximumVoltage - minimumVoltage)
intercept = -slope * minimumVoltage


# Access config file in home directory
def getConfig():
    try:
        home_dir = expanduser("~")

        path = os.path.abspath(os.path.join(home_dir, 'config'))
        file = open(path, "r")
        line = file.readline().rstrip()
        namespace = decrypt(line)
    except:
        HEIFU_INFO('File not found')
        return -1
    return namespace


# Decrypt cipher from config file
def decrypt(encryption):
    key = b64encode(hashlib.sha256(b'BeyondSkyline19').digest())[0:32]
    encryption = b64decode(encryption)
    cipher = AES.new(key, AES.MODE_ECB)
    unpad = lambda s: s[:-ord(s[len(s) - 1:])]
    decripted = unpad(cipher.decrypt(encryption)).decode('utf8').split('\n')[1]
    return decripted


# Get token after login
def getToken():
    url = URL + '/drone/auth'

    payload = 'assetId=' + NAMESPACE + '&password=' + PASSWORD
    headers = {
        'Content-Type': 'application/x-www-form-urlencoded'
    }

    response = requests.request("POST", url, headers=headers, data=payload)
    token = response.text.encode('utf8')
    return token


decripted = getConfig()
ASSETNAME = decripted.split(',')[0]
PASSWORD = decripted.split(',')[1]
NAMESPACE = decripted.split(',')[2]
TOKEN = getToken().strip('\"')

HEIFU_INFO('Asset:: ' + ASSETNAME + ' ID::' + NAMESPACE)
HEIFU_INFO('With the following token: ' + TOKEN)

sio = socketio.Client(reconnection=True,
                      logger=False,
                      engineio_logger=False)


# Callbacks from all subscribers to send information to backend
def cbDisarm(msg):
    sio.emit(NAMESPACE + '/disarm', '')


def cbArm(msg):
    sio.emit(NAMESPACE + '/mavros/cmd/arming', 'value: true')


def cbTakeOff(msg):
    sio.emit(NAMESPACE + '/takeoff', {'msg': True})


def cbTakeOffDiagnostic(msg):
    HEIFU_INFO('Send takeoff ACK-> %s' % str(msg.data))
    sio.emit(NAMESPACE + '/mission/takeoffACK', msg.data)


def cbLand(msg):
    sio.emit(NAMESPACE + '/land', {'msg': True})


def cbLandDiagnostic(msg):
    HEIFU_INFO('Send land ACK-> %s' % str(msg.data))
    sio.emit(NAMESPACE + '/mission/landACK', msg.data)


def cbPose(msg):
    sio.emit(NAMESPACE + '/m/lp/o', {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y,
                                     'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w})
    sio.emit(NAMESPACE + '/m/lp/a', ({'a': msg.pose.position.z}))


def cbVel(msg):
    sio.emit(NAMESPACE + '/m/lp/v', ({'x': msg.twist.linear.x, 'y': msg.twist.linear.y, 'z': msg.twist.linear.z}))


def cbGPS(msg):
    sio.emit(NAMESPACE + '/m/gp/g', {'a': msg.altitude, 'x': msg.latitude, 'y': msg.longitude})


def cbState(msg):
    sio.emit(NAMESPACE + '/m/s', ({'connected': msg.connected, 'armed': msg.armed,
                                   'guided': msg.guided, 'manual_input': msg.manual_input, 'mode': msg.mode,
                                   'system_status': msg.system_status}))


def cbCompressedImage(msg):
    sio.emit(NAMESPACE + '/i', b64encode(msg.data))


def cbStatus(msg):
    sio.emit(NAMESPACE + '/c/s', ({'m': msg.data}))


def cbGpsFix(msg):
    sio.emit(NAMESPACE + '/g/f/m', msg.data)


def cbBattery(msg):
    global lastBatteryValue
    if 'lastBatteryValue' in globals():
        currentVoltage = slope * msg.voltage + intercept
        # HEIFU_INFO('Last-> ' + str(lastBatteryValue) + ' || Current-> ' + str(currentVoltage))
        if lastBatteryValue <= currentVoltage:
            return
        else:
            lastBatteryValue = currentVoltage
            sio.emit(NAMESPACE + '/b', {'p': lastBatteryValue})
    else:  # enter here when battery is null - first time
        currentVoltage = slope * msg.voltage + intercept
        lastBatteryValue = currentVoltage
        sio.emit(NAMESPACE + '/b', {'p': lastBatteryValue})


def cbSatNum(msg):
    sio.emit(NAMESPACE + '/s', ({'n': msg.data}))


def cbMagStatus(msg):
    HEIFU_INFO('Mag message-> %s' % str(msg))
    sio.emit(NAMESPACE + '/m/m/s', msg.data)


def cbMagReport(msg):
    HEIFU_INFO('Mag reporter: %s' % str(msg))
    sio.emit(NAMESPACE + '/m/m/r', {'r': msg.report, 'c': msg.confidence})


def cbFCUErr(msg):
    sio.emit(NAMESPACE + '/m/fcu/e', msg.data)


def cbAcclStatus(msg):
    HEIFU_INFO('%s' % str(msg))
    sio.emit(NAMESPACE + '/m/a/s', msg.data)


def cbAPMInit(msg):
    sio.emit(NAMESPACE + '/m/apm/s', msg.data)


def cbWaypointReached(msg):
    global totalWaypointsMission
    global isMissionStarted
    global waypointsCode
    global waypointCounter

    if 'totalWaypointsMission' in globals():
        waypointCounter += 1
        percentage = 100.0 * (float(waypointCounter) / float(totalWaypointsMission))

        sio.emit(NAMESPACE + '/mission/status',
                 ({'total': totalWaypointsMission, 'reached': waypointCounter, 'percentage': percentage}))
        HEIFU_INFO('Total-> %s || Reached-> %s || Percentage-> %s' % (
        str(totalWaypointsMission), str(waypointCounter), str(percentage)))
        if int(percentage) >= int(100):
            HEIFU_INFO('End Mission')
            sio.emit(NAMESPACE + '/mission/end', '')
            isMissionStarted = False
            if waypointsCode[len(waypointsCode) - 1] == 21:  # Land code
                pubLandMission.publish()
            return

        if int(waypointCounter + 1) >= int(totalWaypointsMission):
            if waypointsCode[waypointCounter] == 21:  # Land code
                HEIFU_INFO('Will do land now')
                # Fake land on mission to stop video recording
                # pubLandMission.publish()
                sio.emit(NAMESPACE + '/mission/landACK', '')


def cbFilename(msg):
    HEIFU_INFO(msg)
    sio.emit(NAMESPACE + '/filename', msg.data)


# Publishers
pubWarning = rospy.Publisher('warning', std_msgs.msg.String, queue_size=10)
pubCmd = rospy.Publisher('heifu/frontend/cmd', geometry_msgs.msg.Twist, queue_size=10)
pubLand = rospy.Publisher('heifu/land', std_msgs.msg.Empty, queue_size=10)
pubTakeoff = rospy.Publisher('heifu/takeoff', std_msgs.msg.Empty, queue_size=10)
pubAuto = rospy.Publisher('heifu/mode_auto', std_msgs.msg.Empty, queue_size=10)
pubRtl = rospy.Publisher('heifu/rtl', std_msgs.msg.Empty, queue_size=10)
pubStopMission = rospy.Publisher('heifu/mission/stop', std_msgs.msg.Empty, queue_size=10)
pubSetPoint = rospy.Publisher('heifu/global_setpoint_converter', geographic_msgs.msg.GeoPose, queue_size=10)
pubCmdGimbal = rospy.Publisher('heifu/frontend/gimbal', geometry_msgs.msg.Twist, queue_size=10)
pubLandMission = rospy.Publisher('heifu/mission/land', std_msgs.msg.Empty, queue_size=10)

# Services
srvCmd = rospy.ServiceProxy('heifu/mavros/cmd/command', mavros_msgs.srv.CommandLong)
srvCmdAck = rospy.ServiceProxy('heifu/mavros/cmd/command_ack', mavros_msgs.srv.CommandAck)
gimbal_service = rospy.ServiceProxy('heifu/gimbal/setAxes', setGimbalAxes)
srvMissionClear = rospy.ServiceProxy('heifu/mavros/mission/clear', mavros_msgs.srv.WaypointClear)
srvMissionPush = rospy.ServiceProxy('heifu/mavros/mission/push', mavros_msgs.srv.WaypointPush)
# srvMissionPull = rospy.ServiceProxy('heifu/mavros/mission/pull',mavros_msgs.srv.WaypointPull)
srvMissionMode = rospy.ServiceProxy('heifu/mavros/set_mode', mavros_msgs.srv.SetMode)
srvArm = rospy.ServiceProxy('heifu/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
srvGimbal = rospy.ServiceProxy('heifu/gimbal/setAxes', setGimbalAxes)


def Subscribers():
    rospy.Subscriber('heifu/disarm', std_msgs.msg.Empty, cbDisarm)
    rospy.Subscriber('heifu/takeoff', std_msgs.msg.Empty, cbTakeOff)
    rospy.Subscriber('heifu/diagnostic/takeoff', Bool, cbTakeOffDiagnostic)
    rospy.Subscriber('heifu/land', std_msgs.msg.Empty, cbLand)
    rospy.Subscriber('heifu/diagnostic/land', Bool, cbLandDiagnostic)
    rospy.Subscriber('heifu/mavros/local_position/pose', geometry_msgs.msg.PoseStamped, cbPose)
    rospy.Subscriber('heifu/mavros/local_position/velocity_local', geometry_msgs.msg.TwistStamped, cbVel)
    rospy.Subscriber('heifu/mavros/global_position/global', sensor_msgs.msg.NavSatFix, cbGPS)
    rospy.Subscriber('heifu/mavros/state', mavros_msgs.msg.State, cbState)
    rospy.Subscriber('heifu/camera/compressed', sensor_msgs.msg.CompressedImage, cbCompressedImage)
    rospy.Subscriber('heifu/c/s', std_msgs.msg.String, cbStatus)
    rospy.Subscriber('heifu/mavros/battery', sensor_msgs.msg.BatteryState, cbBattery)
    rospy.Subscriber('heifu/mavros/global_position/raw/satellites', std_msgs.msg.UInt32, cbSatNum)
    rospy.Subscriber('heifu/g/f/m', std_msgs.msg.Int8, cbGpsFix)
    rospy.Subscriber('heifu/mavros/MagCalibration/status', std_msgs.msg.UInt8, cbMagStatus)
    rospy.Subscriber('heifu/mavros/MagCalibration/report', MagnetometerReporter, cbMagReport)
    rospy.Subscriber('/heifu/mavros/fcuE', std_msgs.msg.String, cbFCUErr)
    rospy.Subscriber('/heifu/mavros/accl/status', std_msgs.msg.String, cbAcclStatus)
    rospy.Subscriber('/heifu/mavros/apm/init', std_msgs.msg.Bool, cbAPMInit)
    rospy.Subscriber('heifu/mavros/mission/reached', WaypointReached, cbWaypointReached)
    rospy.Subscriber('filename', std_msgs.msg.String, cbFilename)


# defines on connect event and prints if connection is established
@sio.event
def connect():
    HEIFU_INFO('connection established')
    time.sleep(2)
    loadConfigFiles()
    global initiated
    if initiated == False:
        Subscribers()
        initiated = True


# defines on disconnect event and prints if connection is lost
@sio.event
def disconnect():
    global pidProcess

    if 'pidProcess' in globals():
        killCommand = "kill -2 " + str(pidProcess)  # Create the kill interrupt command
        output = subprocess.call(['bash', '-c', killCommand])  # Run the process to video recorder

    saveConfigFiles()
    HEIFU_INFO('disconnected from server')


# connects to backend
sio.connect(URLWS + '?source=drone&token=' + TOKEN)


@sio.on(NAMESPACE + '/shutdown/communications')
def onShutdown(msg):
    sio.disconnect()
    HEIFU_INFO('shutdown %s' % str(msg))


# Publish information to drone after receiving from backend
@sio.on('warning')
def onWarning(msg):
    pubWarning.publish()


@sio.on(NAMESPACE + '/frontend/cmd')
def onCmd(msg):
    conversion = heifu_definitions.TwistCv(msg)
    HEIFU_INFO('cmd_vel')
    pubCmd.publish(conversion)


@sio.on(NAMESPACE + '/frontend/gimbal')
def onGimbal(msg):
    HEIFU_INFO('%s' % str(msg))
    conversionGimbal = geometry_msgs.msg.Twist()
    conversionGimbal.angular.z = msg['x'] / 4.0
    gimbalControl = setGimbalAxesRequest()
    gimbalControl.pitch = -msg['y']
    gimbalControl.roll = 0
    gimbalControl.yaw = 0

    srvGimbal(gimbalControl)

    if not conversionGimbal.angular.z == 0:
        pubCmd.publish(conversionGimbal)
        pass


@sio.on(NAMESPACE + '/mavros/command')
def onCommand(msg):
    HEIFU_INFO('Received command: ')
    HEIFU_INFO('%s' % str(msg['command']))
    srvCmd(msg['broadcast'], msg['command'], msg['confirmation'], msg['param1'], msg['param2'], msg['param3'],
           msg['param4'], msg['param5'], msg['param6'], msg['param7'])


@sio.on(NAMESPACE + '/mavros/command_ack')
def onCommandACK(msg):
    HEIFU_INFO('Received command ACK')
    srvCmdAck(msg['command'], msg['result'], msg['progress'], msg['result_param2'])


@sio.on(NAMESPACE + '/frontend/land')
def onLand(msg):
    global isMissionStarted
    pubLand.publish()
    HEIFU_INFO('LAND')
    isMissionStarted = False


@sio.on(NAMESPACE + '/frontend/takeoff')
def onTakeoff(msg):
    global isMissionStarted
    pubTakeoff.publish()
    HEIFU_INFO('TAKEOFF')
    isMissionStarted = False


@sio.on(NAMESPACE + '/mission/push')
def onPush(msg):
    global totalWaypointsMission
    global isMissionStarted
    global waypointsCode
    global waypointCounter

    pubStopMission.publish()
    time.sleep(3)  # Wait for takeoff 3 seconds
    result = srvMissionClear()
    HEIFU_INFO('Clear Mission-> %s' % str(result))
    waypointsCode = []
    waypointCounter = 0

    waypointList = WaypointList()

    # Adding global waypoint (probably to set altitude);
    initialWaypoint = Waypoint()
    initialWaypoint.frame = 0  # global
    initialWaypoint.command = 16  # waypoint
    initialWaypoint.is_current = False
    initialWaypoint.autocontinue = True
    initialWaypoint.param1 = 0
    initialWaypoint.param2 = 0
    initialWaypoint.param3 = 0
    initialWaypoint.param4 = 0
    initialWaypoint.x_lat = 0
    initialWaypoint.y_long = 0
    initialWaypoint.z_alt = 0
    waypointList.waypoints.append(initialWaypoint)

    i = 0
    totalMissionSize = len(msg['waypoints'])
    while i < totalMissionSize:
        conversionWP, codeWP = heifu_definitions.WaypointCv(msg['waypoints'][i])
        waypointList.waypoints.append(conversionWP)
        if codeWP != int(206):  # only save codes different than camera trigger code
            waypointsCode.append(codeWP)
        i += 1
    totalWaypointsMission = len(waypointsCode)
    HEIFU_INFO('Mission Push')
    HEIFU_INFO('%s' % str(waypointList))
    isMissionStarted = False
    srvMissionPush(0, waypointList.waypoints)
    sio.emit(NAMESPACE + '/mission/pushACK', {'mission': msg})


def doMission(msg):
    HEIFU_INFO('/mission/start %s' % str(msg))
    pubTakeoff.publish()
    time.sleep(3)  # Wait for takeoff 3 seconds
    pubAuto.publish()
    sio.emit(NAMESPACE + '/mission/startACK', '')


@sio.on(NAMESPACE + '/mission/start')
def onMissionStart(msg):
    global isMissionStarted

    if 'isMissionStarted' in globals():
        if isMissionStarted:
            return
    isMissionStarted = True
    doMission(msg)


@sio.on(NAMESPACE + '/mission/stop')
def onMissionCancel(msg):
    global isMissionStarted
    HEIFU_INFO('/mission/stop')
    pubStopMission.publish()
    sio.emit(NAMESPACE + '/mission/stopACK', '')
    isMissionStarted = False


@sio.on(NAMESPACE + '/mission/resume')
def onMissionResume(msg):
    global isMissionStarted
    HEIFU_INFO('/mission/resume')
    pubAuto.publish()
    isMissionStarted = True
    sio.emit(NAMESPACE + '/mission/resumeACK', '')


@sio.on(NAMESPACE + '/frontend/rtl')
def onRlt(msg):
    global isMissionStarted
    pubRtl.publish()
    HEIFU_INFO('RTL')
    isMissionStarted = False


@sio.on(NAMESPACE + '/frontend/setpoint')
def onSetpoint(msg):
    HEIFU_INFO('SetPoint')
    waypointPosition = geographic_msgs.msg.GeoPose()
    waypointPosition.position.altitude = msg['altitude']
    waypointPosition.position.latitude = msg['latitude']
    waypointPosition.position.longitude = msg['longitude']
    pubSetPoint.publish(waypointPosition)


def selectStream(msg):
    ####### Legion Camera for testing is the default
    startStreamDef = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc quantizer=25 \
        tune=zerolatency bitrate=4096 ! \"video/x-h264,profile=baseline\"  ! rtph264pay mtu=1300 ! multiudpsink clients=127.0.0.1:" + str(
        msg) + ',' + janusIP + ':' + \
                     str(msg) + " sync=false async=false"
    if jetsonON:
        switcher = {
            "realsense": "gst-launch-1.0 -v v4l2src device=/dev/video" + str(devId) + " ! 'video/x-raw,format=(string)YUY2, width=1280, height=720, framerate=30/1' ! nvvidconv !  \
            nvv4l2h264enc control-rate=0 insert-sps-pps=1 bitrate=2048000 profile=0 ! rtph264pay ! multiudpsink \
            clients=" + janusIP + ':' + str(msg) + ", sync=false async=false",
            "raspy": "gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! \
            nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=" + janusIP + ':' + str(
                msg) + \
                     " sync=false async=false",
            "insta360": "gst-launch-1.0 -v rtspsrc location=rtsp://" + str(insta360IP) + "/live/live latency=0 ! multiudpsink \
            clients=" + janusIP + ':' + str(msg) + " sync=false ",
        }
    else:
        switcher = {
            "realsense": "gst-launch-1.0 -v v4l2src device=/dev/video" + str(devId) + " ! 'video/x-raw,format=(string)YUY2, width=1280, height=720, framerate=30/1' ! videoconvert !  \
            x264enc pass=qual quantizer=28 bitrate=8192 tune=zerolatency  ! \"video/x-h264,profile=baseline\" ! rtph264pay ! multiudpsink \
            clients=" + janusIP + ':' + str(msg) + ", sync=false async=false",
            "raspy": "gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! nvv4l2h264enc bitrate=8000000 \
            insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=" + janusIP + ':' + str(
                msg) + " sync=false async=false",
            "insta360": "gst-launch-1.0 -v rtspsrc location=rtsp://" + str(insta360IP) + "/live/live latency=0 ! multiudpsink \
            clients=" + janusIP + ':' + str(msg) + " sync=false ",
        }
    return switcher.get(camType, startStreamDef)


@sio.on(NAMESPACE + '/port')
def onStream(msg):
    HEIFU_INFO('%s' % str(msg))
    global pidProcess

    ####### RaspyCam
    # PreProd
    # startStream = "gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    # LAN
    # startStream = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8000000 ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,192.168.1.164:" + str(msg) + " sync=false async=false"
    # startStream = "gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,192.168.1.77:" + str(msg) + " sync=false async=false"

    ####### Real Sense
    ##Lan
    # startStream = "gst-launch-1.0 -v v4l2src device=/dev/video3 ! 'video/x-raw,format=(string)YUY2, width=1920, height=1080, framerate=30/1' ! nvvidconv ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    ##PreProd
    # startStream = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! 'video/x-raw,format=(string)YUY2, width=1920, height=1080, framerate=30/1' ! nvvidconv ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"

    ####### Simulation GAZEBO CAMERA
    ##PreProd
    # startStream = "gst-launch-1.0 -v v4l2src device=/dev/video9 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8000000 ! \"video/x-h264,profile=baseline\" ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(
    #   msg) + " sync=false async=false"

    # Preprod Home
    # startStream = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8000000 ! \"video/x-h264,profile=baseline\"  ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,192.168.1.164:" + str(msg) + " sync=false async=false"

    ####### Insta360
    ##Lan
    # startStream = "gst-launch-1.0 -v rtpbin name=rtpbin  rtspsrc location=rtsp://192.168.1.107/live/live latency=20 ! rtpbin.send_rtp_sink_0  rtpbin.send_rtp_src_0 ! udpsink host=192.168.1.73 port=50193 sync=false  rtpbin.send_rtcp_src_0 ! udpsink host=127.0.0.1 port=" + str(msg) + " sync=false async=false"
    ##PreProd
    # startStream = "gst-launch-1.0 -v rtpbin name=rtpbin  rtspsrc location=rtsp://192.168.1.107/live/live latency=20 ! rtpbin.send_rtp_sink_0  rtpbin.send_rtp_src_0 ! udpsink host=192.168.1.73 port=50193 sync=false  rtpbin.send_rtcp_src_0 ! udpsink host=213.141.23.150 port=" + str(msg) + " sync=false async=false"
    ##Transcode Preprod
    # startStream = "gst-launch-1.0 -v rtpbin name=rtpbin  rtspsrc location=rtsp://192.168.1.75/live/live ! rtph264depay ! decodebin ! x264enc ! "video/x-h264,profile=constrained-baseline" ! rtph264pay ! udpsink host=213.141.23.150 port=" + str(msg) + " sync=false async=false"
    # startStream = "gst-launch-1.0 -v rtpbin name=rtpbin  rtspsrc location=rtsp://192.168.1.78/live/live ! rtph264depay ! decodebin ! x264enc ! "video/x-h264,profile=constrained-baseline" ! rtph264pay ! udpsink host=213.63.130.247 port=" + str(msg) + " sync=false async=false"

    startStream = selectStream(msg)
    HEIFU_INFO('%s' % str(startStream))
    output = subprocess.Popen(['bash', '-c', startStream + " & disown"])  # Run the process to video recorder
    pidProcess = output.pid + 1  # Get the PID process
    sio.emit(NAMESPACE + '/portACK', '')


@sio.on(NAMESPACE + '/janusDown')
def stopStream(msg):
    global pidProcess
    if 'pidProcess' in globals():
        killCommand = "kill -2 " + str(pidProcess)  # Create the kill interrupt command
        output = subprocess.call(['bash', '-c', killCommand])  # Run the process to video recorder


@sio.on(NAMESPACE + '/listVideos')
def listVideos():
    list_of_files = glob.glob(VIDEOLIST)
    sio.emit(NAMESPACE + '/listVideos', list_of_files)  # TODO REMOVE DIR from every file


@sio.on(NAMESPACE + '/lastVideo')
def lastVideo():
    list_of_files = glob.glob(VIDEOLIST)  # list all mp4 files
    latest_file = max(list_of_files, key=os.path.getctime)
    file = latest_file.split('/')
    filename = file[len(file) - 1]
    sio.emit(NAMESPACE + '/lastVideo', filename)


@sio.on(NAMESPACE + '/videoRequest')
def videoRequest(videoName):
    global sendindVideo

    if sendindVideo:
        sio.emit(NAMESPACE + '/videoRequestACK', {'sucess': False})
        return
    else:
        sio.emit(NAMESPACE + '/videoRequestACK', {'sucess': True})
        sendindVideo = True

    HEIFU_INFO('%s' % str(videoName))
    fileCompletePath = VIDEOPATH + videoName
    HEIFU_INFO('%s' % str(fileCompletePath))
    hed = {'Authorization': NAMESPACE}
    file = {'file': open(fileCompletePath, 'rb')}
    HEIFU_INFO('Sending video...')
    response = requests.post(UPLOADURL, files=file, headers=hed)
    HEIFU_INFO('Done sending post %s' % str(response.content))
    sendindVideo = False


@sio.on(NAMESPACE + '/playAudio')
def playAudioOnce(audioName):
    hed = {'Authorization': TOKEN}
    response = requests.get(DOWNLOADAUDIOURL, headers=hed)
    base = '/home/heifu/'
    dir = 'audios'
    file = audioName['filename']
    saveFile = os.path.join(base, dir, file)
    folderPath = os.path.join(base, dir)

    if not os.path.exists(folderPath):  # Verify if the folder exist
        os.mkdir(folderPath)

    # getFolderSize(folderPath)

    if not path.exists(saveFile):
        response = requests.get(DOWNLOADAUDIOURL + '/' + file, headers=hed)
        f = open(saveFile, 'w')
        f.write(response.content)
        f.close()
        HEIFU_INFO('Received audio file')

    audioProcessfile = '/home/heifu/audios/process_file'
    if path.exists(audioProcessfile):
        f = open(audioProcessfile, 'r')
        os.system('kill -9 ' + str(f.read()))
        os.remove(audioProcessfile)
        f.close()

    # Play Video
    os.system(
        'sh /home/heifu/heifu_ws/src/heifu/heifu_interface/playAudio.sh --base ' + base + ' --dir ' + dir + ' --file ' + file + ' &')


@sio.on(NAMESPACE + '/playAudioLoop')
def playAudioLoop(audioName):
    hed = {'Authorization': TOKEN}
    response = requests.get(DOWNLOADAUDIOURL, headers=hed)
    base = '/home/heifu/'
    dir = 'audios'
    file = audioName['filename']
    saveFile = os.path.join(base, dir, file)
    folderPath = os.path.join(base, dir)

    if not os.path.exists(folderPath):  # Verify if the folder exist
        os.mkdir(folderPath)

    if not path.exists(saveFile):
        response = requests.get(DOWNLOADAUDIOURL + '/' + file, headers=hed)
        f = open(saveFile, 'w')
        f.write(response.content)
        f.close()
        HEIFU_INFO('Received audio file')

    audioProcessfile = '/home/heifu/audios/process_file'
    if path.exists(audioProcessfile):
        f = open(audioProcessfile, 'r')
        os.system('kill -9 ' + str(f.read()))
        os.remove(audioProcessfile)
        f.close()

    # Play Video
    os.system(
        'sh /home/heifu/heifu_ws/src/heifu/heifu_interface/playAudio.sh --base ' + base + ' --dir ' + dir + ' --file ' + file + ' --loop &')


@sio.on(NAMESPACE + '/stopAudio')
def stopAudio(audioName):
    audioProcessfile = '/home/heifu/audios/process_file'
    f = open(audioProcessfile, 'r')
    os.system('kill -9 ' + str(f.read()))
    if path.exists(audioProcessfile):
        os.remove(audioProcessfile)
    f.close()
    HEIFU_INFO('Stop audio file')


@sio.on(NAMESPACE + '/logrequest')
def logRequest(logmsg):
    HEIFU_INFO('/logrequest %s' % str(logmsg))
    sio.emit(NAMESPACE + '/lr', logmsg)


def getFolderSize(folder):
    total_size = os.path.getsize(folder)
    for item in os.listdir(folder):
        itempath = os.path.join(folder, item)
        if os.path.isfile(itempath):
            total_size += os.path.getsize(itempath)
        elif os.path.isdir(itempath):
            total_size += getFolderSize(itempath)
    total_size = total_size * 1e-9  # convert to Gg
    HEIFU_INFO('Used space in audio folder-> %s' % str(total_size * 1e-9))

    if total_size > 5:
        for i in listdir(folder):
            os.remove(os.path.join(folder, i))
            break


def saveConfigFiles():
    global isMissionStarted
    global totalWaypointsMission
    global waypointCounter
    global lastBatteryValue
    global waypointsCode

    home_dir = expanduser("~")
    filePath = os.path.abspath(os.path.join(home_dir, 'configParams'))

    if path.exists(filePath):
        os.remove(filePath)

    f = open(filePath, 'w')
    if 'isMissionStarted' in globals():
        f.write('isMissionStarted\n' + str(isMissionStarted) + '\n')
    else:
        f.write('isMissionStarted\nnull\n')

    if 'totalWaypointsMission' in globals():
        f.write('totalWaypointsMission\n' + str(totalWaypointsMission) + '\n')
    else:
        f.write('totalWaypointsMission\nnull\n')

    if 'waypointCounter' in globals():
        f.write('waypointCounter\n' + str(waypointCounter) + '\n')
    else:
        f.write('waypointCounter\nnull\n')

    if 'lastBatteryValue' in globals():
        f.write('lastBatteryValue\n' + str(lastBatteryValue) + '\n')
    else:
        f.write('lastBatteryValue\nnull\n')

    if 'waypointsCode' in globals():
        f.write('waypointsCode\n' + str(waypointsCode) + '\n')
    else:
        f.write('waypointsCode\nnull\n')
    f.close()


def loadConfigFiles():
    global isMissionStarted
    global totalWaypointsMission
    global waypointCounter
    global lastBatteryValue
    global waypointsCode

    home_dir = expanduser("~")
    filePath = os.path.abspath(os.path.join(home_dir, 'configParams'))

    if not path.exists(filePath):
        return

    f = open(filePath, 'r')
    allParametersConfig = f.read().splitlines()

    HEIFU_INFO('Load Parameters-> %s' % str(allParametersConfig))

    index = 1
    if allParametersConfig[index] != "null":
        isMissionStarted = eval(allParametersConfig[index])
    index += 2
    if allParametersConfig[index] != "null":
        totalWaypointsMission = int(allParametersConfig[index])
    index += 2
    if allParametersConfig[index] != "null":
        waypointCounter = int(allParametersConfig[index])
    index += 2
    if allParametersConfig[index] != "null":
        lastBatteryValue = float(allParametersConfig[index])
    index += 2
    if allParametersConfig[index] != "null":
        strWaypoints = allParametersConfig[index]
        strWaypoints = strWaypoints.replace('[', '')
        strWaypoints = strWaypoints.replace(']', '')
        waypointsCode = map(int, strWaypoints.split(','))

    f.close()


if __name__ == '__main__':
    # Init node
    try:
        sendindVideo = False
        rospy.init_node('interface', anonymous=True)
        rospy.Rate(20)
        rospy.spin()
    finally:
        saveConfigFiles()
