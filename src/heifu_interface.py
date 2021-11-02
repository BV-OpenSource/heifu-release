#!/usr/bin/env python

# ROS
import rospy
import std_msgs
import geometry_msgs.msg
import geographic_msgs.msg
import sensor_msgs.msg
import mavros_msgs
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, CommandLong, CommandBool, SetMode
from mavros_msgs.msg import WaypointList, Waypoint, WaypointReached, MagnetometerReporter, CommandCode
from gimbal.msg import setGimbalAxes

# NFZ
# from shapely.geometry import Point
# from shapely.geometry.polygon import Polygon
# import nfz_stop
# import formulas

# from tf.transformations import euler_from_quaternion
#from sensors_node.msg import rfid_humtemp

# General
# import json
# import time
import log

# import glob
# from threading import Thread, Timer
# from time import sleep
# import sys
from os import path

######
import commands
import state
import report
import nfz
import mission
import actuators
import sensors
import gstreamer
import utils
import time
logger = log.UAVLogger(path.basename(__file__), writeFile=True)
# if (len(sys.argv)>1):
#    ROSNAME = sys.argv[1]
#    configname = '-' + ROSNAME
# else:
#    ROSNAME = 'heifu0'
#    configname = ''


class UAVposition:
    latitude = None  # 0 #39.1212552265913
    longitude = None  # 0 #-9.379234313964846


class GlobalPosition:
    latitude = None  # 0 #39.1212552265913
    longitude = None  # 0 #-9.379234313964846
    altitude = None

UAV_INFO = logger.info
UAV_WARNING = logger.warning
UAV_ERROR = logger.error
UAV_EXCEPTION = logger.exception
UAV_DEBUG = logger.debug

class UAV:
    def __init__(self, vehicle, socket, enableNFZ, isSimulation):
        self.isInited = False
        self.socket = socket
        self.socket.ros_interface = self
        self.vehicle = vehicle
        self.logger = logger

        self.enable_nfz = enableNFZ
        self.no_Takeoff = False
        self.uav_position = GlobalPosition()
        self.simulation = isSimulation

        self._takeoff       = commands.Takeoff(self)
        self._land          = commands.Land(self)
        self._arm           = commands.Arm(self)
        self._pose          = state.Pose(self)
        self._velocity      = state.Velocity(self)
        self._gps           = state.GPS(self)
        self._nfz           = nfz.NFZ(self)
        self._state         = state.State(self)
        self._magreport     = report.MagReport(self)
        self._battery       = state.Battery(self)
        self._command       = commands.Command(self)
        self._mission       = mission.Mission(self)
        self._rtl           = commands.RTL(self)
        self._setpoint      = mission.Setpoint(self)
        self._statusreport  = report.StatusReport(self)
        self._gimbal        = actuators.Gimbal(self)
        self._gst           = gstreamer.GSTreamer(self)
        self._file          = utils.File(self)
        # self._rfid     = sensors.RFID(self)

        self.Subscribers()
        self.Publishers()
        self.Services()
        #time.sleep(5) # Sleep for 5 seconds

        rospy.init_node('interface', anonymous=True)
        self.isInited = True


    # Subscribers
    def Subscribers(self):
        # Takeoff
        # rospy.Subscriber('takeoff', std_msgs.msg.Empty, self._takeoff.cbTakeOff)
        rospy.Subscriber('diagnostic/takeoff',
                         std_msgs.msg.Bool, self._takeoff.cbTakeOffDiagnostic)

        # Land
        # rospy.Subscriber('land', std_msgs.msg.Empty, self._land.cbLand)
        rospy.Subscriber('diagnostic/land',
                         std_msgs.msg.Bool, self._land.cbLandDiagnostic)

        # Arm/Disarm
        rospy.Subscriber('disarm',
                         std_msgs.msg.Empty, self._arm.cbDisarm)
        rospy.Subscriber('arm',
                         std_msgs.msg.Empty, self._arm.cbArm)

        # Pose
        rospy.Subscriber('mavros/local_position/pose',
                         geometry_msgs.msg.PoseStamped, self._pose.cbPose)

        # Velocity
        rospy.Subscriber('mavros/local_position/velocity_local',
                         geometry_msgs.msg.TwistStamped, self._velocity.cbVel)

        # GPS
        rospy.Subscriber('mavros/global_position/global',
                         sensor_msgs.msg.NavSatFix, self._gps.cbGPS)
        rospy.Subscriber('mavros/global_position/raw/satellites',
                         std_msgs.msg.UInt32, self._gps.cbSatNum)
        rospy.Subscriber('g/f/m',
                         std_msgs.msg.Int8, self._gps.cbGpsFix)

        # State
        rospy.Subscriber('mavros/state',
                         mavros_msgs.msg.State, self._state.cbState)

        # Battery
        rospy.Subscriber('battery',
                         sensor_msgs.msg.BatteryState, self._battery.cbBattery)

        # Status Report
        rospy.Subscriber('c/s',
                         std_msgs.msg.String, self._statusreport.cbStatus)
        rospy.Subscriber('mavros/fcuE',
                         std_msgs.msg.String, self._statusreport.cbFCUErr)
        rospy.Subscriber('mavros/accl/status',
                         std_msgs.msg.String, self._statusreport.cbAcclStatus)
        rospy.Subscriber('mavros/apm/init',
                         std_msgs.msg.Bool, self._statusreport.cbAPMInit)

        # Mag Report
        rospy.Subscriber('mavros/MagCalibration/status',
                         std_msgs.msg.UInt8, self._magreport.cbMagStatus)
        rospy.Subscriber('mavros/MagCalibration/report',
                         MagnetometerReporter, self._magreport.cbMagReport)

        # Mission
        rospy.Subscriber('mavros/mission/reached',
                         mavros_msgs.msg.WaypointReached, self._mission.cbWaypointReached)
        rospy.Subscriber('mavros/mission/waypoints',
                         mavros_msgs.msg.WaypointList, self._mission.cbWaypointList)

        # RFID
        # rospy.Subscriber(self.vehicle + 'sensors/data', rfid_humtemp, self._rfid.cbSensorRfid)

        # File
        rospy.Subscriber(self.vehicle + 'filename',
                         std_msgs.msg.String, self._file.cbFilename)

        pass

    def Publishers(self):
        # Arm
        self.pubArm = rospy.Publisher(
            'arm', std_msgs.msg.Empty, queue_size=10)

        # Loiter
        self.pubLoiter = rospy.Publisher(
            'loiter', std_msgs.msg.Empty, queue_size=10)

        # Takeoff
        self.pubTakeoff = rospy.Publisher(
            'takeoff', std_msgs.msg.UInt8, queue_size=10)

        # Land
        self.pubLand = rospy.Publisher(
            'land', std_msgs.msg.Empty, queue_size=10)

        # RTL
        self.pubRtl = rospy.Publisher(
            'rtl', std_msgs.msg.Empty, queue_size=10)

        # Mission
        self.pubStopMission = rospy.Publisher(
            'mission/stop', std_msgs.msg.Empty, queue_size=10)
        self.pubStartMission = rospy.Publisher(
            'mission/start', std_msgs.msg.Empty, queue_size=10)
        self.pubAuto = rospy.Publisher(
            'mode_auto', std_msgs.msg.Empty, queue_size=10)
        self.pubGuided = rospy.Publisher(
            'mode_guided', std_msgs.msg.Empty, queue_size=10)

        # Setpoint
        self.pubSetPoint = rospy.Publisher(
            'waypointsManager/setpoint', geographic_msgs.msg.GeoPoint, queue_size=10)
        
        # Gimbal
        self.pubCmd = rospy.Publisher(
            'frontend/cmd', geometry_msgs.msg.Twist, queue_size=10)
        self.pubCmdGimbal = rospy.Publisher(
            'frontend/gimbal', geometry_msgs.msg.Twist, queue_size=10)

        # RFID
        self.pubSensorStart = rospy.Publisher(
            'sensors/start', std_msgs.msg.Empty, queue_size=10)
        self.pubSensorStop = rospy.Publisher(
            'sensors/stop', std_msgs.msg.Empty, queue_size=10)

        # Gimbal
        self.pubGimbal = rospy.Publisher(
            'gimbal/setAxes', setGimbalAxes, queue_size=10)

        # Cameras - Gstreamer
         #stream to an online platform
        self.pubStartTStream = rospy.Publisher(
            'gstreamer/online/start', std_msgs.msg.Empty, queue_size=10)
        self.pubStopTStream = rospy.Publisher(
            'gstreamer/online/stop', std_msgs.msg.Empty, queue_size=10)
        # store to file
        self.pubStartRecording = rospy.Publisher(
            'gstreamer/recording/start', std_msgs.msg.Empty, queue_size=10)
        self.pubStopRecording = rospy.Publisher(
            'gstreamer/recording/stop', std_msgs.msg.Empty, queue_size=10)
        # take photo
        self.pubTakePhoto = rospy.Publisher(
            'gstreamer/photo', std_msgs.msg.Empty, queue_size=10)


        # Publishers
        #####self.pubWarning = rospy.Publisher('warning', std_msgs.msg.String, queue_size=10)
        #####self.pubLandMission = rospy.Publisher(self.vehicle + '/mission/land', std_msgs.msg.Empty, queue_size=10)

    def Services(self):
        # Command
        self.srvCmd = rospy.ServiceProxy(
            'mavros/cmd/command', mavros_msgs.srv.CommandLong)
        self.srvCmdAck = rospy.ServiceProxy(
            'mavros/cmd/command_ack', mavros_msgs.srv.CommandAck)
        self.srvArm = rospy.ServiceProxy(
            'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        self.srvSetMode = rospy.ServiceProxy(
            'mavros/set_mode', mavros_msgs.srv.SetMode)

        # Mission
        self.srvMissionClear = rospy.ServiceProxy(
            'mavros/mission/clear', mavros_msgs.srv.WaypointClear)
        self.srvMissionPush = rospy.ServiceProxy(
            'mavros/mission/push', mavros_msgs.srv.WaypointPush)

        pass

    def start(self):
        rospy.Rate(20)
        rospy.spin()
