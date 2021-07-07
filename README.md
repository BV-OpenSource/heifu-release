# GPU Voxels ROS

ROS package to integrate the library [GPU Voxels](https://github.com/fzi-forschungszentrum-informatik/gpu-voxels)

## Building the Library

To initialize, fetch and checkout any nested submodules, you can use the foolproof command.

```bash
git submodule update --init --recursive
cd gpu-voxels
```

Install the [library dependencies](https://github.com/fzi-forschungszentrum-informatik/gpu-voxels#install-dependencies).

Follow the following commands instead to compiling:

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../export -DCMAKE_C_COMPILER=/usr/bin/gcc-8 -DCUDA_TOOLKIT_INCLUDE=/usr/local/cuda-10.2/include -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2
make install
```

Then add the follow to the ~/.bashrc file to update the environment variables.

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(rospack find gpu_voxels_ros)/gpu-voxels/export/lib
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(rospack find gpu_voxels_ros)/gpu-voxels/export/share
```

Source the file `source ~/.bashrc` or close and reopen the terminal.

Proceed with the normal ROS package compilation.

## Dependencies:

* [tf](http://wiki.ros.org/tf)
* [roscpp](http://wiki.ros.org/roscpp)
* [pcl_ros](http://wiki.ros.org/pcl_ros)
* [nodelet](http://wiki.ros.org/nodelet)
* [pluginlib](http://wiki.ros.org/pluginlib)
* [geometry_msgs](http://wiki.ros.org/geometry_msgs)
* CUDA 7.5, 8.0, 9.x, 10.0 or 10.2
* PCL
* OpenNI
* Boost
* TinyXML (libtinyxml-dev)

##  Publishers:

* GPU_Voxels/map  [[std_msgs/UInt64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/UInt64.html)]
* GPU_Voxels/PCD [[sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)]
* GPU_Voxels/offset  [[geometry_msgs/PointStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html)]
* GPU_Voxels/MutexMap  [[std_msgs/UInt64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/UInt64.html)]

##  Subscribers:

* GPU_Voxels/pcdCalib  [[std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]
* GPU_Voxels/cleanMap [[std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]
* pointcloud/depth/color/points [[sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)]
