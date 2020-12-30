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
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear
from mavros_msgs.msg import WaypointList, Waypoint, WaypointReached
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

#URL = 'http://192.168.1.164:3000'
#URLWS = 'http://192.168.1.164:3000'
URL = 'https://beyond-skyline-backend.preprod.pdmfc.com'
ciURLWS = 'https://beyond-skyline-backend.preprod.pdmfc.com'
UPLOADURL = URL + '/drone/uploadVideo'
DOWNLOADAUDIOURL = URL + '/drone/downloadAudio'
VIDEOPATH = '/home/heifu/Videos/'
VIDEOLIST = '/home/heifu/Videos/*.mp4'

global pidProcess
global TOKEN
global sendindVideo
global totalWaypointsMission
global lastBatteryValue
global initiated
iniiated = False

# URL = 'http://192.168.1.77:3000'
# URLWS = 'http://192.168.1.77:3000'
# ip graca: 85.243.249.239

# Access config file in home directory
def getConfig():
    try:
        home_dir = expanduser("~")
        
        path = os.path.abspath(os.path.join(home_dir, 'camera'))
        file = open(path, "r")
        line = file.readline().rstrip()
        namespace = decrypt(line)
    except:
        print('File not found')
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
    
    url = URL + '/camera/auth'

    payload = 'name=' + NAMESPACE + '&password=' + PASSWORD + '&organization=' + ORG
    headers = {
    'Content-Type': 'application/x-www-form-urlencoded'
    }

    response = requests.request("POST", url, headers=headers, data = payload)
    token = response.text.encode('utf8')
    return token


decripted = getConfig()
NAMESPACE = decripted.split(',')[0]
PASSWORD = decripted.split(',')[1]
ORG = decripted.split(',')[2]
TOKEN = getToken().strip('\"')

print('Asset:: ' + NAMESPACE)
print('With the following token: ' + TOKEN)

sio = socketio.Client(reconnection=True,
                      logger=False,
                      engineio_logger=False)


# Callbacks from all subscribers to send information to backend
def cbDisarm(msg):
    sio.emit(NAMESPACE+'/disarm', '')


def cbArm(msg):
    sio.emit(NAMESPACE+'/mavros/cmd/arming', 'value: true')

def cbTakeOff(msg):
    sio.emit(NAMESPACE+'/takeoff', {'msg': True})


def cbTakeOffDiagnostic(msg):
    print('Send takeoff ACK-> ', msg.data)
    sio.emit(NAMESPACE+'/mission/takeoffACK', msg.data)


def cbLand(msg):
    sio.emit(NAMESPACE+'/land', {'msg': True})


def cbLandDiagnostic(msg):
    print('Send land ACK-> ', msg.data)
    sio.emit(NAMESPACE+'/mission/landACK', msg.data)

def cbPose(msg):
    sio.emit(NAMESPACE+'/m/lp/o', {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y,
                                   'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w})
    sio.emit(NAMESPACE+'/m/lp/a', ({'a': msg.pose.orientation.z}))


def cbVel(msg):
    sio.emit(NAMESPACE+'/m/lp/v', ({'x': msg.twist.linear.x, 'y': msg.twist.linear.y, 'z': msg.twist.linear.z}))


def cbGPS(msg):
    sio.emit(NAMESPACE+'/m/gp/g', {'a': msg.altitude, 'x': msg.latitude, 'y': msg.longitude})


def cbState(msg):
    sio.emit(NAMESPACE+'/m/s', ({'connected': msg.connected, 'armed': msg.armed,
                                 'guided': msg.guided, 'manual_input': msg.manual_input, 'mode': msg.mode,
                                 'system_status': msg.system_status}))


def cbCompressedImage(msg ):
    sio.emit(NAMESPACE+'/i', b64encode(msg.data))


def cbStatus(msg):
    sio.emit(NAMESPACE+'/c/s', ({'m': msg.data}))


def cbGpsFix(msg):
    sio.emit(NAMESPACE+'/g/f/m',  msg.data)


def cbBattery(msg):
    global lastBatteryValue

    if 'lastBatteryValue' in globals():
        if round(lastBatteryValue) == round(msg.percentage*100):
            return

    lastBatteryValue = msg.percentage*100
    sio.emit(NAMESPACE+'/b', {'p': lastBatteryValue})


def cbSatNum(msg):
    sio.emit(NAMESPACE+'/s', ({'n': msg.data}))


def cbWaypointReached(msg):
    global totalWaypointsMission

    if 'totalWaypointsMission' in globals():
        percentage = 100.0*(float(msg.wp_seq)/float(totalWaypointsMission))
        sio.emit(NAMESPACE + '/mission/status', ({'total': totalWaypointsMission, 'reached': msg.wp_seq, 'percentage': percentage}))
        print('Total-> ' + str(totalWaypointsMission) +
              ' || Reached-> ' + str(msg.wp_seq) + ' || Percentage-> ' + str(percentage))

# Publishers
pubWarning = rospy.Publisher('warning', std_msgs.msg.String, queue_size=10)
pubCmd = rospy.Publisher('heifu/frontend/cmd', geometry_msgs.msg.Twist, queue_size=10)
pubLand = rospy.Publisher('heifu/land', std_msgs.msg.Empty, queue_size=10)
pubTakeoff = rospy.Publisher('heifu/takeoff', std_msgs.msg.Empty, queue_size=10)
pubAuto = rospy.Publisher('heifu/mode_auto', std_msgs.msg.Empty, queue_size=10)
pubRtl = rospy.Publisher('heifu/rtl', std_msgs.msg.Empty, queue_size=10)
pubSetPoint = rospy.Publisher('heifu/global_setpoint_converter', geographic_msgs.msg.GeoPose, queue_size=10)
pubCmdGimbal = rospy.Publisher('heifu/frontend/gimbal', geometry_msgs.msg.Twist, queue_size=10)

# Services
# srvMissionClear = rospy.ServiceProxy('heifu/mavros/mission/clear',mavros_msgs.srv.WaypointClear)
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
    rospy.Subscriber('heifu/camera/compressed', sensor_msgs.msg.CompressedImage , cbCompressedImage)
    rospy.Subscriber('heifu/c/s', std_msgs.msg.String , cbStatus)
    rospy.Subscriber('heifu/mavros/battery', sensor_msgs.msg.BatteryState, cbBattery)
    rospy.Subscriber('heifu/mavros/global_position/raw/satellites', std_msgs.msg.UInt32, cbSatNum)
    rospy.Subscriber('heifu/g/f/m', std_msgs.msg.Int8, cbGpsFix)
    rospy.Subscriber('heifu/mavros/mission/reached', WaypointReached, cbWaypointReached)

# defines on connect event and prints if connection is established
@sio.event
def connect():
    print('connection established')
    time.sleep(2)
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

    print('disconnected from server')

# connects to backend
sio.connect( URLWS + '?source=camera&token=' + TOKEN)

# Publish information to drone after receiving from backend
@sio.on('warning')
def onWarning(msg):
    pubWarning.publish()

@sio.on(NAMESPACE+'/frontend/cmd')
def onCmd(msg):
    conversion = heifu_definitions.TwistCv(msg)
    print('cmd_vel')
    pubCmd.publish(conversion)

@sio.on(NAMESPACE+'/frontend/gimbal')
def onGimbal(msg):
    print(msg)
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

@sio.on(NAMESPACE+'/frontend/land')
def onLand(msg):
    pubLand.publish()
    print('LAND')

@sio.on(NAMESPACE+'/frontend/takeoff')
def onTakeoff(msg):
    pubTakeoff.publish()
    print('TAKEOFF')

@sio.on(NAMESPACE+'/mission/push')
def onPush(msg):
    global totalWaypointsMission
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
    totalWaypointsMission = len(msg['waypoints'])
    while i < totalWaypointsMission:
        conversionWP = heifu_definitions.WaypointCv(msg['waypoints'][i])
        waypointList.waypoints.append(conversionWP)
        i += 1
    print('Mission Push')
    print(waypointList)
    srvMissionPush(0, waypointList.waypoints)
    sio.emit(NAMESPACE + '/mission/pushACK', {'mission': msg})

@sio.on(NAMESPACE+'/mission/start')
def onMissionStart(msg):
    print('/mission/start', msg)
    pubTakeoff.publish()
    time.sleep(2)
    pubAuto.publish()
    sio.emit(NAMESPACE + '/mission/startACK', '')

@sio.on(NAMESPACE+'/frontend/rtl')
def onRlt(msg):
    pubRtl.publish()
    print('RTL')

@sio.on(NAMESPACE+'/frontend/setpoint')
def onSetpoint(msg):
    print('SetPoint')
    waypointPosition = geographic_msgs.msg.GeoPose()
    waypointPosition.position.altitude = msg['altitude']
    waypointPosition.position.latitude = msg['latitude']
    waypointPosition.position.longitude = msg['longitude']
    pubSetPoint.publish(waypointPosition)

@sio.on(NAMESPACE+'/port')
def onStream(msg):
    print(msg)
    global pidProcess
    
    ####### RaspyCam
    #PreProd
    #startStream = "gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    #LAN
    #startStream = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8000000 ! \"video/x-h264,profile=baseline\" ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,192.168.1.164:" + str(msg) + " sync=false async=false"
    #startStream = "gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,192.168.1.77:" + str(msg) + " sync=false async=false"
    ####### Real Sense
    ##Lan
    #startStream = "gst-launch-1.0 -v v4l2src device=/dev/video3 ! 'video/x-raw,format=(string)YUY2, width=1920, height=1080, framerate=30/1' ! nvvidconv ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    ##PreProd
    #startStream = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! 'video/x-raw,format=(string)YUY2, width=1920, height=1080, framerate=30/1' ! nvvidconv ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    ####### Simulation GAZEBO CAMERA
    ##PreProd
    #startStream = "gst-launch-1.0 -v v4l2src device=/dev/video9 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8000000 ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    #Preprod Home
    startStream = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8000000 ! \"video/x-h264,profile=baseline\" ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,192.168.1.164:" + str(msg) + " sync=false async=false"
    #startStream = "gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8000000 ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    #output = subprocess.Popen(['bash', '-c', startStream + " & disown"]) #Run the process to video recorder
    #startStream = "gst-launch-1.0 -v v4l2src device=/dev/video2 ! 'video/x-raw,format=(string)YUY2, width=1920, height=1080, framerate=30/1' ! nvvidconv ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    ####### Simulation GAZEBO CAMERA
    ##PreProd
    #startStream = "gst-launch-1.0 -v v4l2src device=/dev/video9 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoconvert ! x264enc pass=qual quantizer=20 tune=zerolatency bitrate=8000000 ! rtph264pay mtu=1400 ! multiudpsink clients=127.0.0.1:5000,213.141.23.150:" + str(msg) + " sync=false async=false"
    output = subprocess.Popen(['bash','-c', startStream + " & disown"]) #Run the process to video recorder
    pidProcess = output.pid + 1 #Get the PID process
    sio.emit(NAMESPACE+'/portACK', '')

@sio.on(NAMESPACE+'/listVideos')
def listVideos():
    list_of_files = glob.glob(VIDEOLIST)
    sio.emit(NAMESPACE+'/listVideos', list_of_files) #TODO REMOVE DIR from every file

@sio.on(NAMESPACE+'/lastVideo')
def lastVideo():
    list_of_files = glob.glob(VIDEOLIST) # list all mp4 files
    latest_file = max(list_of_files, key=os.path.getctime)
    file = latest_file.split('/')
    filename = file[len(file)-1]
    sio.emit(NAMESPACE+'/lastVideo', filename)

@sio.on(NAMESPACE+'/videoRequest')
def videoRequest(videoName):
    global sendindVideo

    if sendindVideo:
        sio.emit(NAMESPACE + '/videoRequestACK', {'sucess': False})
        return
    else:
        sio.emit(NAMESPACE + '/videoRequestACK', {'sucess': True})
        sendindVideo = True

    print(videoName)
    fileCompletePath = VIDEOPATH + videoName
    print(fileCompletePath)
    hed = {'Authorization': TOKEN}
    file = {'file': open(fileCompletePath, 'rb')}
    print('Sending video...')
    response = requests.post(UPLOADURL, files=file, headers=hed)
    print('Done sending post', response.content)
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
    
    if not os.path.exists(folderPath): # Verify if the folder exist
        os.mkdir(folderPath)
        
    # getFolderSize(folderPath)
    
    if not path.exists(saveFile):
        response = requests.get(DOWNLOADAUDIOURL+'/'+file, headers=hed)
        f = open(saveFile, 'w')
        f.write(response.content)
        f.close()
        print('Received audio file')
    
    audioProcessfile = '/home/heifu/audios/process_file'
    if path.exists(audioProcessfile):
        f = open(audioProcessfile, 'r')
        os.system('kill -9 ' + str(f.read()))
        os.remove(audioProcessfile)

    #Play Video
    os.system('sh /home/heifu/heifu_ws/src/heifu/heifu_interface/playAudio.sh --base ' + base + ' --dir ' + dir + ' --file ' + file + ' &')

@sio.on(NAMESPACE + '/playAudioLoop')
def playAudioLoop(audioName):
    hed = {'Authorization': TOKEN}
    response = requests.get(DOWNLOADAUDIOURL, headers=hed)
    base = '/home/heifu/'
    dir = 'audios'
    file = audioName['filename']
    saveFile = os.path.join(base, dir, file)
    folderPath = os.path.join(base, dir)
    
    if not os.path.exists(folderPath): #Verify if the folder exist
        os.mkdir(folderPath)
    
    if not path.exists(saveFile):
        response = requests.get(DOWNLOADAUDIOURL+'/'+file, headers=hed)
        f = open(saveFile, 'w')
        f.write(response.content)
        f.close()
        print('Received audio file')
    
    audioProcessfile = '/home/heifu/audios/process_file'
    if path.exists(audioProcessfile):
        f = open(audioProcessfile, 'r')
        os.system('kill -9 ' + str(f.read()))
        os.remove(audioProcessfile)
    
    #Play Video
    os.system('sh /home/heifu/heifu_ws/src/heifu/heifu_interface/playAudio.sh --base ' + base + ' --dir ' + dir + ' --file ' + file + ' --loop &')

@sio.on(NAMESPACE + '/stopAudio')
def stopAudio(audioName):
    audioProcessfile = '/home/heifu/audios/process_file'
    f = open(audioProcessfile, 'r')
    os.system('kill -9 ' + str(f.read()))
    if path.exists(audioProcessfile):
        os.remove(audioProcessfile)
    print('Stop audio file')
    
def getFolderSize(folder):
    total_size = os.path.getsize(folder)
    for item in os.listdir(folder):
        itempath = os.path.join(folder, item)
        if os.path.isfile(itempath):
            total_size += os.path.getsize(itempath)
        elif os.path.isdir(itempath):
            total_size += getFolderSize(itempath)
    total_size = total_size*1e-9 #convert to Gg
    print('Used space in audio folder-> '+str(total_size*1e-9))
    
    if total_size > 5:
        for i in listdir(folder):
          os.remove(os.path.join(folder, i))
          break

if __name__ == '__main__':
    # Init node
    sendindVideo = False
    rospy.init_node('interface', anonymous=True)
    rospy.Rate(20)
    rospy.spin()
