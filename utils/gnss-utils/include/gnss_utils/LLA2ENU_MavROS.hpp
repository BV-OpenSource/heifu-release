#ifndef LLA2ENU_MAVROS_HPP
#define LLA2ENU_MAVROS_HPP

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <std_srvs/Trigger.h>
#include "gnss_utils/GNSS_utils.h"

class LLA2ENU_MavROS
{
    public:
        LLA2ENU_MavROS();

        void Run();

    private:
        ros::NodeHandle nh;

        ros::ServiceClient srv;

        geographic_msgs::GeoPoint home;

        bool home_defined = false;

        ros::Publisher converted_pub;
        ros::Publisher homeConvertedPub;

        ros::Subscriber goal_sub;
        ros::Subscriber home_sub;

        void cbGoal(const geographic_msgs::GeoPointStamped &msg);
        void cbHome(const geographic_msgs::GeoPointStamped &msg);
};

#endif // LLA2ENU_MAVROS_HPP
