#!/bin/bash

# Call the running script with the received arguments
python /home/heifu/heifu_ws/src/heifu/heifu_interface/playAudio.py "$@" &

# Get value of --name from second set of arguments
processFiles="/home/heifu/audios/process_file"
echo $! >> "${processFiles}"
