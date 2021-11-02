# HEIFU Repository

## Pre-requisites
**Install ROS acording to your distribuiton:**

ROS Instalation page can be found [here](http://wiki.ros.org/ROS/Installation).

**Install all dependencies:**
```bash
sudo apt install python-catkin-tools
sudo apt install -y ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-joy-teleop ros-$ROS_DISTRO-mavlink libgeographic-dev ros-$ROS_DISTRO-geographic-msgs gdal-bin libgdal-dev wget geographiclib-tools libgeographic-dev gstreamer1.0-tools libgstreamer1.0-dev
wget https://gitlab.pdmfc.com/drones/ros1/heifu/-/raw/master/heifu_scripts_firmware/loader.py
sudo cp loader.py /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages/roslaunch/
rm loader.py
```

## Installation

**NOTE:** It's recommended too add a SSH key to your gitlab account.

### Start by cloning
```bash
cd <your_workspace_path>/src
git clone -b newVersion git@gitlab.pdmfc.com:drones/ros1/heifu-uav/heifu.git
cd heifu
git submodule update --init --recursive
```

### Compile your workspace

Run the script in:
```bash
sudo ./control/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

#### In case that GPU is **NOT** present or NOT wanted to be used:
-   ```bash
    catkin config --blacklist collision_avoidance gpu_voxels_ros planner planners_manager rrt
    ```

Follow instructions on packages:
- interface/gcs-interface
- sensing/gpu_voxels_ros (ignore if [last step](#in-case-that-gpu-is-not-present-or-not-wanted-to-be-used) was taken)

At last, compile your workspace:
```
catkin build
```

### Source Your Workspace:

```bash
echo 'source <your_workspace_path>/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## HELP - How to get going

```bash
roslaunch heifu-bringup heifu_bringup.launch argVehicle:="heifu" argID:=0 argSimulation:=false
```

## Package description
**GCS Interface:**
Interface for converting commands and information from and to a remote control station, respectively.

**Waypoint Manager:**
Reads mission files from the UAV and sends them to the respective nodes. Allows following a mission in guided mode.

**GNSS Utils:**
Auxiliary to the Waypoints manager package. Converts global coordinates to local coordinates.

**Planners:**
Responsible for finding a collision-free path to the desired waypoint.

**Collision Avoidance:**
Deals with the information from the perception sensors and ensures the safety of the UAV.

**Priority Manager:**
Receives all setpoints from the respective packages. Responsible for managing the priorities of each package, sending always the command with the highest priority to the navigation controller.

**Mavros Commands:**
Responsible for general flight commands handling, such as takeoff, land, and mode changes.

**Navigation Controller:**
Responsible for controlling and completing given waypoints. Can be used with position setpoints or velocity setpoints. Guarantees that the requested position is achieved. Can receive a velocity bypass setpoint.

**Mavros:**
Responsible for the comunication between ROS and the flight controller firmware. Uses the MAVlink protocol.
