#include "gnss_utils/LLA2ENU.hpp"

LLA2ENU::LLA2ENU():nh("~")
{
	std::string ns = ros::this_node::getNamespace();

	converted_pub = nh.advertise<geometry_msgs::Pose>(ns + "/gnss_utils/converted_coordinates", 10);

	goal_sub = nh.subscribe(ns + "/gnss_utils/goal_coordinates", 10, &LLA2ENU::cbGoal, this);
	home_sub = nh.subscribe(ns + "/gnss_utils/home_coordinates", 10, &LLA2ENU::cbHome, this);
}

void LLA2ENU::Run(){
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void LLA2ENU::cbGoal(const geometry_msgs::Pose &msg){
	geometry_msgs::Pose converted;
	if(home_defined){
		GNSS_utils::LLAtoENU(msg.position.x, msg.position.y, msg.position.z,
							 home.position.x, home.position.y, home.position.z,
							 converted.position.x, converted.position.y, converted.position.z);
		converted_pub.publish(converted);
	}
}

void LLA2ENU::cbHome(const geometry_msgs::Pose &msg){
	home = msg;
	home_defined = true;
}
