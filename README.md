# Installation

**ROS packages dependences:**
* gazebo-ros
* mavros [here](https://gitlab.pdmfc.com/drones/ros1/control/mavros)
* mavlink
* geographic-msgs
* tf2-eigen
* control-toolbox
* GNSS_utils [here](https://gitlab.pdmfc.com/drones/ros1/utils/gnss-utils)


**GeographicLib dependeces:**

```bash
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
```

Run the script in:
[install_geographiclib_datasets.sh](https://github.com/mavlink/mavros/blob/master/mavros/scripts/install_geographiclib_datasets.sh)


Compile this directory in your ROS workspace.

Then copy the already configured files to the *mavros* directory:

```bash
cd ${YOUR_ROS_WORKSPACE}/src/heifu/heifu_scripts_firmware
cp apm_config.yaml $(rospack find mavros)/launch/
cp apm.launch $(rospack find mavros)/launch/
cp node.launch $(rospack find mavros)/launch/
```

**NOTE:** Adjust the yaml loader script to deal with the argument substitution in the *roslaunch* of yaml files:
```bash
cd ${YOUR_ROS_WORKSPACE}/src/heifu/heifu_scripts_firmware
cp loader.py /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/
```
Dont forget to add [gimbal](https://gitlab.pdmfc.com/drones/ros1/gimbals/arrishobby-zhaoyun) and [mavros](https://gitlab.pdmfc.com/drones/ros1/mavros.git) to the source folder and run `catkin build`

To run the nodes with a real drone:

Just launch - Heifu_bringup
```sh
roslaunch heifu_bringup heifu_bringup.launch
```

**Launch Input arguments:**
```sh
    argSafety - Activates the safety zone mode and run the node heifu_safety.
    argTakeOffAltitude - Defines the Takeoff altitude.
    argSim - Activates the simulation mode. Loads Gazebo and a pre-configured world with the drone HEIFU.
    argSecredas - Activates the Secredas Usecases. (Needs extra packages secredas).
    argUseCase - Defines witch usecase of Secredas will be loaded.
    argPlanners - Loads the Planners nodes with static colision avoidance (Needs extra packages Planners).
```

------

**Heifu Simulation:**

First make sure that the ArduPilot firmware is correctly installed, by following the steps described [here](https://gitlab.pdmfc.com/drones/heifu/wikis/Install-and-Configure-ArduPilot-Firmware).


To open the simulation:

1st terminal - Launch Heifu_bringup
```sh
roslaunch heifu_bringup heifu_bringup.launch argSim:=true
```

2nd terminal - Launch ArduPilot firmware
```sh
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-heifu -I1
```
	
	
------

# Packages

## Heifu Bringup
Responsible for load all packages.

------

## Heifu Description
This package contains the drone robot HEIFU and the worlds for the simulation.

------

## Heifu Diagnostic
Responsible for verify the GPS fix state of the drone and send the information to the application.

#### Subscribers: 

* /diagnostics ![diagnostic_msgs/DiagnosticArray](http://docs.ros.org/melodic/api/diagnostic_msgs/html/msg/DiagnosticArray.html)

#### Publishers:

* /heifu/g/f/m ![std_msgs/Int8](http://docs.ros.org/melodic/api/std_msgs/html/msg/Int8.html)
------

## Heifu Interface
This package is the bridge between the ROS and the application.

#### Subscribers: 

* /heifu/disarm ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/takeoff ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/land ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/mavros/local_position/pose ![geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)
* /heifu/mavros/local_position/velocity_local ![geometry_msgs/TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html)
* /heifu/mavros/global_position/global ![sensor_msgs/NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)
* /heifu/mavros/state ![mavros_msgs/State](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/State.html)
* /heifu/c/s ![std_msgs/String](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)
* /heifu/mavros/battery ![sensor_msgs/BatteryState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html)
* /heifu/mavros/global_position/raw/satellites ![std_msgs/UInt32](http://docs.ros.org/melodic/api/std_msgs/html/msg/UInt32.html)
* /heifu/g/f/m ![std_msgs/String](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)

#### Publishers:

* /warning ![std_msgs/String](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)
* /heifu/frontend/cmd ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/land ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/takeoff ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/mode_auto ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/rtl ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/global_setpoint_converter ![geographic_msgs/GeoPose](http://docs.ros.org/melodic/api/geographic_msgs/html/msg/GeoPose.html)

#### Services: 

* /heifu/mavros/mission/clear ![mavros_msgs/WaypointPush](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/WaypointPush.html)
* /heifu/mavros/set_mode ![mavros_msgs/SetMode](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/SetMode.html)
* /heifu/mavros/cmd/arming ![mavros_msgs/CommandBool](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/CommandBool.html)
------

## Heifu Mavros
This is the main package to control the drone. Responsible for message conversions and commands between the packages and the mavros node.

#### Subscribers: 

* /heifu/takeoff ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/land ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/disarm ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/cmd_vel ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/xbox_vel_raw ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/mavros/local_position/pose ![geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)
* /heifu/rtl ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/land_vel_raw ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/simulation_raw ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/frontend/cmd ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/mode_auto ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/global_setpoint_converter ![geographic_msgs/GeoPose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/GeoPose.html)
* /heifu/mission/start ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/mission/stop ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)

#### Publishers: 
* /heifu/mavros/setpoint_velocity/cmd_vel ![geometry_msgs/TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html)
* /heifu/xbox_vel ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/land_vel ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/simulation ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu//g/f/m ![std_msgs/Int8](http://docs.ros.org/melodic/api/std_msgs/html/msg/Int8.html)
* /heifu/mavros/setpoint_position/local ![geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)

#### Services:

* /heifu/mavros/cmd/arming ![mavros_msgs/CommandBool](http://docs.ros.org/melodic/api/mavros_msgs/html/srv/CommandBool.html)
* /heifu/mavros/set_mode ![mavros_msgs/SetMode](http://docs.ros.org/melodic/api/mavros_msgs/html/srv/SetMode.html)
* /heifu/mavros/takeoff ![mavros_msgs/CommandTOL](http://docs.ros.org/melodic/api/mavros_msgs/html/srv/CommandTOL.html)
------

## Heifu Msgs
This package contain the messages and services necessary to work.

------

## Heifu Safety
If loaded, this package controls the flight area and velocity limit of the drone to a safety real demonstration.

#### Subscribers: 

* /heifu/mavros/local_position/pose ![geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)
* /heifu/xbox_vel ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)

#### Publishers:

* /heifu/xbox_vel_safety ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)

#### Services:

* /heifu/EnableSafetyFence ![heifu_msgs/EnableSafety](http://docs.ros.org/melodic/api/heifu_msgs/html/srv/EnableSafety.html)

------

## Heifu Simple Waypoint
This package receive a setpoint and make the drone fly pointed to desired position.

#### Subscribers: 

* /heifu/mission/setpoint ![geometry_msgs/Pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html)
* /heifu/mavros/local_position/pose ![geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)

#### Publishers:

* /heifu/mission/start ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/mission/stop ![std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html)
* /heifu/mavros/setpoint_position/local ![geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)
------

## Heifu Tools
Responsible for convert the velocity command messages.

#### Subscribers: 

* /heifu/joy ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)

#### Publishers:

* /heifu/dummy_vel ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
* /heifu/xbox_vel_raw ![geometry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
------
