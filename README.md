# GSTREAMER ROS Node

Creates 4 gstreamer pipelines which run simultaneously.

## Main Pipeline - Standard video settings & Tee

Applies aspect ratio and other video parameters which are common to all streams.
(TODO: Add stabilization)

![image](./src/pipe_diagrams/TeePipe.png)

## Pipeline 1 - Local stream

Local stream to be shown in RC display.

![image](./src/pipe_diagrams/pipe_t1.png)

## Pipeline 2 - Online stream

Online stream sent to Janus.

![image](./src/pipe_diagrams/pipe_t2.png)

## Pipeline 3 - Recording

High quality stream which saves .mp4 video to disk.

![image](./src/pipe_diagrams/pipe_t3.png)

## Pipeline 4 - Photo (single run)

Saves a single .jpg snapshot to disk

![image](./src/pipe_diagrams/pipe_t4.png)
