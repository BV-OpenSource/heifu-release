# RRT

## Description

Package to plan a route for Heifu using RRT.

## Running

### In order to use simulation:

Terminal 1
```bash
roscore
```

Terminal 2
```bash
cd {ardupilot_PATH}/ArduCopter
./../Tools/autotest/sim_vehicle.py -f gazebo-heifu -v ArduCopter
```

Terminal 3
```bash
roslaunch heifu_description gazebo.launch
```

------

If in simulation, run this in the simulation computer. If in real situation, run it in the Heifu.

Terminal 1
```bash
roslaunch heifu_mavros heifu_mavros.launch
```
Terminal 2
```bash
roslaunch octomap_server RRT_octomap_mapping.launch
```

### Usage

The node can be launched by calling the [PlannerManager](../../planners_manager/tree/master/launch/Planners_Manager.launch) launch file with the [argPlannerType](../../planners_manager#arguments) set as 1.
```bash
roslaunch planners_manager Planners_Manager.launch argPlannerType:=1
```
------
