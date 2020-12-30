# Description

This package acts as an interface between ROS, a Backend and a Web-RTC server. It authenticates the UAV with the backend, and then transmits, via Websocket, information
like current position, current speed, GPS state, among others. While doing this, after beeing authenticated, it also recieves inputs from the Frontend, namingly movement 
instructions (cmd_vel), missions, takeoff and landing commands. Furthermore, it establishes a RTP stream to a Web-RTC server (Janus) that is later redirected to the 
Frontend.

Before starting the interface, make sure that you have ROS and Mavros running and install the following libs.

`pip install "python-socketio[client]"==4.4.0`

`pip install python-engineio==3.11.1`

To run the interface, move to the directory and run:

```bash
python heifu_interface.py
```

#### Heifu definitions

This is used when a message that is sent from the backend has to be structured in a specific message type for the drone to digest.
In this case, we receive a waypoint through the socket and on the definitions we initialize a variable with the Waypoint format and use the data that was received to convert it to Waypoint.

This should be used for other formats for the drone to digest.

#### Backend URLS: 

URL = 'https://beyond-skyline-backend.preprod.pdmfc.com'
URLWS = 'https://beyond-skyline-backend.preprod.pdmfc.com'


#### Subscribers: 

* rospy.Subscriber('heifu/disarm', std_msgs.msg.Empty, cbDisarm)
* rospy.Subscriber('heifu/takeoff', std_msgs.msg.Empty, cbTakeOff)
* rospy.Subscriber('heifu/land', std_msgs.msg.Empty, cbLand)
* rospy.Subscriber('heifu/mavros/local_position/pose', geometry_msgs.msg.PoseStamped, cbPose)
* rospy.Subscriber('heifu/mavros/local_position/velocity_local', geometry_msgs.msg.TwistStamped, cbVel)
* rospy.Subscriber('heifu/mavros/global_position/global', sensor_msgs.msg.NavSatFix, cbGPS)
* rospy.Subscriber('heifu/mavros/state', mavros_msgs.msg.State , cbState)
* rospy.Subscriber('heifu/camera/compressed', sensor_msgs.msg.CompressedImage , cbCompressedImage)
* rospy.Subscriber('heifu/c/s', std_msgs.msg.String , cbStatus)
* rospy.Subscriber('heifu/mavros/battery', sensor_msgs.msg.BatteryState, cbBattery)
* rospy.Subscriber('heifu/mavros/global_position/raw/satellites', std_msgs.msg.UInt32, cbSatNum)
* rospy.Subscriber('heifu/g/f/m', std_msgs.msg.String, cbGpsFix)

#### Publishers:

* pubWarning = rospy.Publisher('warning', std_msgs.msg.String, queue_size=10)
* pubCmd = rospy.Publisher('heifu/frontend/cmd', geometry_msgs.msg.Twist, queue_size=10)
* pubLand = rospy.Publisher('heifu/land', std_msgs.msg.Empty, queue_size=10)
* pubTakeoff = rospy.Publisher('heifu/takeoff', std_msgs.msg.Empty, queue_size=10)
* pubAuto = rospy.Publisher('heifu/mode_auto', std_msgs.msg.Empty, queue_size=10)
* pubRtl = rospy.Publisher('heifu/rtl', std_msgs.msg.Empty, queue_size=10)
* pubSetPoint = rospy.Publisher('heifu/global_setpoint_converter', geographic_msgs.msg.GeoPose, queue_size=10)

#### Services 

* srvMissionPush = rospy.ServiceProxy('heifu/mavros/mission/push',mavros_msgs.srv.WaypointPush)
* srvMissionMode = rospy.ServiceProxy('heifu/mavros/set_mode',mavros_msgs.srv.SetMode)
* srvArm = rospy.ServiceProxy('heifu/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

#### WebSocket Subscribers (They send information to the drone): 

* @sio.on('warning')
* @sio.on(NAMESPACE+'/frontend/cmd')
* @sio.on(NAMESPACE+'/frontend/land')
* @sio.on(NAMESPACE+'/frontend/takeoff')
* @sio.on(NAMESPACE+'/mission/push')
* @sio.on(NAMESPACE+'/mission/start')
* @sio.on(NAMESPACE+'/frontend/rtl')
* @sio.on(NAMESPACE+'/frontend/setpoint')
* @sio.on(NAMESPACE+'/port')

#### Callbacks from all Subscribers - WebSocket Publishers (They send information to the backend): 

* def cbDisarm(msg):
    sio.emit(NAMESPACE+'/disarm','' )
* def cbArm(msg):
    sio.emit(NAMESPACE+'/mavros/cmd/arming','value: true')
* def cbTakeOff(msg):
    sio.emit(NAMESPACE+'/takeoff','' )
* def cbLand(msg):
    sio.emit(NAMESPACE+'/land','' )
* def cbPose(msg):
    sio.emit(NAMESPACE+'/m/lp/o', {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y, 'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w})
    sio.emit(NAMESPACE+'/m/lp/a', ({'a': msg.pose.orientation.z}))
* def cbVel(msg):
    sio.emit(NAMESPACE+'/m/lp/v', ({'x' : msg.twist.linear.x, 'y' : msg.twist.linear.y, 'z' : msg.twist.linear.z}))
* def cbGPS(msg):
    sio.emit(NAMESPACE+'/m/gp/g', {'a': msg.altitude , 'x': msg.latitude , 'y': msg.longitude})
* def cbState(msg):
    sio.emit(NAMESPACE+'/m/s', ({'connected': msg.connected , 'armed': msg.armed , 'guided': msg.guided, 'manual_input': msg.manual_input, 'mode': msg.mode, 'system_status': msg.system_status }))
* def cbCompressedImage(msg ):
    sio.emit(NAMESPACE+'/i', b64encode(msg.data))
* def cbStatus(msg):
    sio.emit(NAMESPACE+'/c/s', ({'m': msg.data}))
* def cbGpsFix(msg):
    sio.emit(NAMESPACE+'/g/f/m',  msg.data )
* def cbBattery(msg):
    sio.emit(NAMESPACE+'/b',{'p': (msg.percentage * 100)} )
* def cbSatNum(msg):
    sio.emit(NAMESPACE+'/s',({'n': msg.data}) )

#### GSTREAMER (only one of the following works at a time, the other must be commented)

* For a Raspberry Cam on Jetson Nano:
    startStream = "gst-launch-1.0 -e nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1' ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients={DESIRED IP}:{DESIRED PORT} " sync=false async=false"

* For a RealSense Cam on Jetson Nano:
    startStream = "gst-launch-1.0 -v v4l2src device=/dev/video3 ! 'video/x-raw,format=(string)YUY2, width=1920, height=1080, framerate=30/1' ! nvvidconv ! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true ! rtph264pay mtu=1400 ! multiudpsink clients={DESIRED IP}:{DESIRED PORT} " sync=false async=false"