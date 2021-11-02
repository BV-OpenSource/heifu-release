# Collision  Avoidance

This package is responsible to avoid collision using a [GPU Voxels map](https://gitlab.pdmfc.com/drones/ros1/sensors/gpu_voxels_ros/-/tree/nodelets) so its installation is recommended.

## Dependencies

* [roscpp](http://wiki.ros.org/roscpp)
* [mavros_msgs](https://gitlab.pdmfc.com/drones/ros1/control/mavros/-/tree/5cbd156ec439a00c4de2bfc7215aa412da79573e/mavros_msgs)
* [nodelet](http://wiki.ros.org/nodelet)
* [pluginlib](http://wiki.ros.org/pluginlib)
* [uav_msgs](https://gitlab.pdmfc.com/drones/ros1/heifu-uav/utils/-/tree/newMessageDefinition/uav_msgs)

## Publishers

* collisionAvoidance/emergencyStop  [[geometry_msgs/TwistStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistStamped.html)}
* planners/collision/pause  [[std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]
* planners/collision/start  [[std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]
* planners/updateSafetyDistance  [[std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]
* visualization_marker  [[visualization_msgs/Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html)]

## Subscribers

* GPU_Voxels/MutexMap [[std_msgs/UInt64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/UInt64.html)]
* GPU_Voxels/map [[std_msgs/UInt64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/UInt64.html)]
* GPU_Voxels/offset [[geometry_msgs/PointStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html)]
* mavros/local_position/pose [[geometry_msgs/PoseStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)]
* mavros/local_position/velocity_local [[geometry_msgs/TwistStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistStamped.html)]
* planners/local_position/setpoint [[uav_msgs/UAVWaypoint](https://gitlab.pdmfc.com/drones/ros1/heifu-uav/utils/-/raw/newMessageDefinition/uav_msgs/msg/UAVWaypoint.msg)]
* waypointsManager/goalWaypoint [[uav_msgs/UAVWaypoint](https://gitlab.pdmfc.com/drones/ros1/heifu-uav/utils/-/raw/newMessageDefinition/uav_msgs/msg/UAVWaypoint.msg)]

## Usage

To use the Collision Avoidance node isolated:
```bash
roslaunch collision_avoidance CollisionAvoidance_nodelets.launch
```

To use Collision Avoidance functionality and all the other node needed:
```bash
roslaunch collision_avoidance CollisionAvoidanceBringup.launch
```
