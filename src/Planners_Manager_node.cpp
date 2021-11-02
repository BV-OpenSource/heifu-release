#include "Planners_Manager/Planners_Manager.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Planners_Manager_node");

	PlannersManager *Client = new PlannersManager();
	Planner *Planners = Client->getPlanner();
	Planners->Setup();
	Planners->Run();
	return 0;
}
