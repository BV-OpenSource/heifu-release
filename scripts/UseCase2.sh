echo "Initializing Dummy Simulation!"

gnome-terminal -x bash -c "source ~/heifu_ws/devel/setup.bash; roslaunch heifu_description gazebo.launch"

sleep 3

gnome-terminal -x bash -c "source ~/heifu_ws/devel/setup.bash; roslaunch heifu_mavros heifu_mavros.launch argID:=2 xi:=-30 Y:=-1.57079632676"
 
sleep 3

gnome-terminal -x bash -c "source ~/heifu_ws/devel/setup.bash; cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-heifu -I2"

sleep 3
gnome-terminal -x bash -c "source ~/heifu_ws/devel/setup.bash; roslaunch heifu_sim heifu_sim.launch argID:=2"

sleep 1

gnome-terminal -x bash -c "source ~/heifu_ws/devel/setup.bash; roslaunch heifu_mavros heifu_mavros.launch argID:=1 yi:=-30"

sleep 3

gnome-terminal -x bash -c "source ~/heifu_ws/devel/setup.bash; cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-heifu -I1"

sleep 3

gnome-terminal -x bash -c "source ~/heifu_ws/devel/setup.bash; roslaunch heifu_sim_sensors heifu_sim_sensors.launch argID:=1 argUseCase2:=1"