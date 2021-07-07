#ifndef PRIORITY_MANAGER_HPP
#define PRIORITY_MANAGER_HPP

#include <ros/ros.h>
#include <ros/console.h>

#include <iterator>
#include <map>
#include <XmlRpcValue.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <uav_msgs/UAVWaypoint.h>

#include <ros/message_event.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "msg_helpers_hpp/msg_listing.hpp"

namespace PM
{

    class Priority_manager
    {
    public:

        Priority_manager();
        virtual ~Priority_manager (){};

        void run();

    private:
        ros::NodeHandle n;

        // Subscribers

        // Services Clients

        // Publishers
        ros::Publisher pubSetpoint;

        // Parameters
        double paramNodeRate;

        std::string paramPubTopicSetpoint;

        // Variables
        typedef struct SourceEntry {
            std::string topic;
            uint8_t type_id;
            double  timeout;
            uint8_t priority;
        } EntryStruct_;

        typedef struct EntryInfo {
            EntryStruct_ data;
            ros::Time last_read;
            ros::Subscriber subscriber;
        } EntryInfoStruct_;

        typedef std::map<std::string, EntryInfoStruct_> EntryMap_;

        EntryMap_ EntryMap;
        std::string lockHolder;

        // Functions
        void cbHeifuMsgsHeifuWaypointmsg(const ros::MessageEvent<uav_msgs::UAVWaypoint const>& event, const std::string &topic);
        void cbGeometryMsgsPoseStampedmsg(const ros::MessageEvent<geometry_msgs::PoseStamped const>& event, const std::string &topic);
        void cbGeometryMsgsTwistStampedmsg(const ros::MessageEvent<geometry_msgs::TwistStamped const>& event, const std::string &topic);

        void parseTopics(ros::NodeHandle *nh, std::string rosparamName);
        void createSubscribers(ros::NodeHandle *nh);
        bool isLockHolder(std::string topic_name);

    };

}

#endif
