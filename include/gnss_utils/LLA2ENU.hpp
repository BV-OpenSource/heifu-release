#ifndef LLA2ENU_HPP
#define LLA2ENU_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "gnss_utils/GNSS_utils.h"

class LLA2ENU
{
	public:
		LLA2ENU();

		void Run();

		void cbGoal(const geometry_msgs::Pose &msg);
		void cbHome(const geometry_msgs::Pose &msg);
	private:
		ros::NodeHandle nh;

		geometry_msgs::Pose home;

		bool home_defined = false;

		ros::Publisher converted_pub;

		ros::Subscriber goal_sub;
		ros::Subscriber home_sub;
};

#endif // LLA2ENU_HPP
