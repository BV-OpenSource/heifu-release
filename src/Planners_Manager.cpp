#include "Planners_Manager/Planners_Manager.hpp"

PlannersManager::PlannersManager():nh("~") {
	PlannersManager(nh, nh);
}

PlannersManager::PlannersManager(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):nh(private_nh_),shm_nh(nh_){
	std::string ns = ros::this_node::getNamespace();

	nh.param<int>("paramPlannerType", paramPlannerType, 0);

	// Client explicitly creates classes according to type
	if (paramPlannerType == 1){
		Planners = new RRT3D::RRT(private_nh_, nh_);
	}
	/*else if (paramPlannerType == 2)
		Planners = new PRM();*/
	else{
		ROS_ERROR("Planner not defined or not found.");
		Planners = nullptr;
	}
}

Planner *PlannersManager::getPlanner()
{
	return  Planners;
}
