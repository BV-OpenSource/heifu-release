#!/bin/bash

# Call the running script with the received arguments
python3 /home/heifu/heifu_ws/src/heifu/heifu_interface/cameraStab.py "$@" &

# Get value of --name from second set of arguments
processFiles="/home/heifu/heifu_ws/src/heifu/heifu_interface/process_file"
echo $! >> "${processFiles}"