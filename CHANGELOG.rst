^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heifu
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.4 (2020-12-30)
------------------

0.7.2 (2020-12-17)
------------------
* Add changelog files.
* Remove ros_utils from all packages.
* Contributors: André Filipe

0.7.1 (2020-08-28)
------------------
* Really make sure every package has the same version number.
* Make sure every package has the same version number.
* Removed stabilize files (now in Janus)
* fixed xml
* initial commit
* placed master options
* pipeline changes
* edited options
* added new realsense pipeline for jetson
* Added project full path
* camera stabilize v1.0
* Update heifu_interface.py
* Update heifu_interface.py
* added filename subscriber and missionland publisher to streamrecorder
* #14 automatic download config file from usb
* Update heifu_interface.py
* Update README.md
* save and close when log
* format logs
* Added time to logs
* Added time to logs
* Add option to save file
* add hour min and second to the file name
* save all print logs
* Merge branch 'ref#13-refactorIDs' into 'master'
  Ref#13 refactor ids
  See merge request drones/ros1/heifu!5
* merge master
* ident
* merge & minor fix
* Merge branch 'master' into ref#13-refactorIDs
* Update heifu_interface.py
* Merge branch 'master' into ref#13-refactorIDs
* Update heifu_interface.py
* #13 change login and topic names to utilize IDs instead of drone Name
* Merge branch 'master' of gitlab.pdmfc.com:drones/ros1/heifu
* The ros subscribers were incrementing every time backend would disconnect and reconnect
* give the right percentages values
* Update heifu_interface.py
* back to *100 battery
* Merge branch 'janusDisconnect' into 'master'
  Janus disconnect
  See merge request drones/ros1/heifu!4
* Added dictionary for endpoint IPs
* reverted changes to other files
* Corrected a few typos
* initial commit
* Merge branch 'ref#12-multipleDrones' into 'master'
  Ref#12 multiple drones
  See merge request drones/ros1/heifu!3
* uncomment preprod IP
* #12 added organization for login
* camera token with organization
* #12 Added org to login to allow multiple drones with same name
* added commandAck back
* Merge branch 'master' of gitlab.pdmfc.com:drones/ros1/heifu
* Fixed an issue with the drones battery state
* ident
* Fix bug heifu_interface.py - send lr all info and update with new calibration method
* Fix bug heifu_interface.py - send lr all info and update with new calibration method
* Fix bug heifu_interface.py - send lr all info
* Fix bug - changed orientation to position to sent right altitude value
* add logrequest socket
* comment video stream
* Merge remote-tracking branch 'origin/master'
* refs#11 - Added camera trigger option
* Update README.md
* refs#11 - Create a config file to save and read global parameters when a drone disconnect
* add print when calibrating pixhawk's mag
* heifu_interface mission version 1.0
* refs#27 - changed topics stop and resume names
* refs#27 - clear mission and resume mission options
* refs#27 - cancel mission if takeoff or land fail
* indent code
* refs#9 and refs#10 - check current waypoint in mission mode and just send cancel current mission option, respectively
* refs#9 and refs#10 - check current waypoint in mission mode and just send cancel current mission option, respectively
* Merge branch 'master' of /home/jmcarvalho/tese_doutoramento_ws/src/heifu with conflicts.
* Merge remote-tracking branch 'origin/master'
* cancel current mission option
* Merge branch 'calibration' into 'master'
  Calibration
  See merge request drones/ros1/heifu!2
* merged with master
* #3 Receive feedback from APM calibration and transmit to the platform
* changed empty message on land and takeoff to a bool
* refs#9 and refs#10 - check current waypoint in mission mode and just send battery information when changed, respectively
* #3 Added Report of compass calibration
* takeoff and land acks
* update heifu audio interfacce
* Added feedback from compass
* update heifu audio interfacce
* play multiple audios
* update heifu audio interfacce
* change methods name
* Update README.md
* fix in print
* #3 Added calibration services
* Merge branch 'master' of gitlab.pdmfc.com:drones/ros1/heifu
* Added pipeline to stream the Simulation camera to application; Reduce YAW velocity to get a better control;
* Update heifu_interface.py
* Update playAudio.py
* Merge branch 'master' of https://gitlab.pdmfc.com/drones/ros1/heifu
* play normal audio, loop and stop audio
* Fix raspcam launch
* Update heifu_interface.py
* play audio by given path
* Fix gimbal value
* Added gimbal launch control
* Added gimbal control
* Gimbal control added
* Small bug fix on convert control
* fix audio
* added audio download
* Merge branch 'video' into 'master'
  Added functions to upload videos to backend
  See merge request drones/ros1/heifu!1
* Added functions to upload videos to backend
* Heifu interface fix
* Simple Waypoint and guimbal control added to interface
* Fix auto respawn MAVROS Apm node.
* Update README.md
* Packages and updates cleaned
* Add README.md
* Contributors: André, Filipe, Guilherme Rolo, João Madeira, Ricardo Sacoto Martins, dsilva, falmeida, jmadeira, jmcarvalho, rsmartins
