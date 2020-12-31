#!/bin/bash

#Directory for where the file is copy
SRC="/home/heifu/"
DEVICE="$1" # the device name

NUMBER="${DEVICE: -1}"

if [ "$NUMBER" = "1" ]
then
	exit 0
fi

#Create folder for symlink
mkdir -p /tmp/"$DEVICE"
mount /dev/"$DEVICE"1 /tmp/"$DEVICE"/

#Verify if config file exists
if [ -f /tmp/"$DEVICE"/config ]; then
    #Copy the config file for home folder
    cd /tmp/"$DEVICE"/ && rsync -av config "$SRC"config
    #Give permission to read config
	chown -R "heifu" "$SRC"config
else 
    echo "$FILE does not exist."
fi

#Unmount and remove symlink folder
umount -l /tmp/"$DEVICE"
rm -r /tmp/"$DEVICE"

exit 0
