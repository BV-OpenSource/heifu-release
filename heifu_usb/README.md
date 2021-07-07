# HEIFU USB

Script to copy Configuration file from USB and insert in Home directory

Setup :
-----------

#### General setup :
Please add that the file `99-heifu.rules` in `/etc/udev/rules.d` to run script when an usb drive is plugged.
Add usb_heifu.sh to the home folder.
Run `sudo udevadm control --reload-rules` to update udev rules
Run `chmod +x usb_heifu.sh`