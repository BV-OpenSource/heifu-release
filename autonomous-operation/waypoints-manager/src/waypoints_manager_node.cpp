#include "waypoints_manager/waypoints_manager.hpp"

int main(int argc, char **argv){
	ros::init(argc, argv, "waypoints_manager_node");
	Waypoints_Manager Waypoints_Manager_node;
	Waypoints_Manager_node.Run();
	return 0;
}
