#!/bin/bash
#GAZEBO: Needs to be running first
#roslaunch heifu_description gazebo.launch 

#variables:
droneName=$1
iD=${2:-1}
xPos=${3:-0}
yPos=${4:-0}

gnome-terminal --tab --title="$droneName" --command="bash -c 'source ~/.bashrc; roslaunch heifu_bringup heifu_bringup.launch argNamespace:=$droneName argID:=$iD xi:=$xPos yi:=$yPos argSim:=true gazebo:=false; $SHELL'"
sleep 20
gnome-terminal --tab --title="$droneName FIRMWARE" --command="bash -c 'source ~/.bashrc; sim_vehicle.py -v ArduCopter -f gazebo-heifu -I$iD; $SHELL'" 
sleep 30
gnome-terminal --tab --title="$droneName Interface" --command="bash -c 'source ~/.bashrc; python ~/heifu_ws/src/heifu/heifu_interface/heifu_interface.py $droneName; $SHELL'"
exit
