// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christian Jülg
 * \date    2015-08-07
 * \author  Andreas Hermann
 * \date    2016-12-24
 *
 * This demo calcuates a distance field on the pointcloud
 * subscribed from a ROS topic.
 * Two virtual meausrement points are places in the scene
 * from which the clearance to their closest obstacle from the live pointcloud
 * is constantly measured (and printed on terminal).
 *
 * Place the camera so it faces you in a distance of about 1 to 1.5 meters.
 *
 * Start the demo and then the visualizer.
 * Example parameters:  ./build/bin/distance_ros_demo -e 0.3 -f 1 -s 0.008  #voxel_size 8mm, filter_threshold 1, erode less than 30% occupied  neighborhoods
 *
 * To see a small "distance hull":
 * Right-click the visualizer and select "Render Mode > Distance Maps Rendermode > Multicolor gradient"
 * Press "s" to disable all "distance hulls"
 * Press "Alt-1" an then "1" to enable drawing of SweptVolumeID 11, corresponding to a distance of "1"
 *
 * To see a large "distance hull":
 * Press "ALT-t" two times, then "s" two times.
 * You will see the Kinect pointcloud inflated by 10 Voxels.
 * Use "t" to switch through 3 slicing modes and "a" or "q" to move the slice.
 *
 */
//----------------------------------------------------------------------
#include <GpuVoxelsRos/GpuVoxelsRos.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gpu_voxels_ros_node");

    icl_core::logging::initialize(argc, argv);

    GPUVoxelsROS GpuVoxelsObj;
    GpuVoxelsObj.Setup();
    GpuVoxelsObj.Run();

    LOGGING_INFO(Gpu_voxels, "shutting down" << endl);

    return 0;
}

