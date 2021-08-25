#include "priority_manager/Priority_manager.hpp"

using namespace PM;

Priority_manager::Priority_manager():n("~") {
    std::string ns = ros::this_node::getNamespace();

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Parameters
    n.param<double>(     "paramNodeRate",         paramNodeRate,         100.0);
    n.param<std::string>("paramPubTopicSetpoint", paramPubTopicSetpoint, "/priority_setpoint");

    // Initialize
    lockHolder = "";
    parseTopics(&n, "entries");
    createSubscribers(&n);

    // Publishers
    pubSetpoint = n.advertise <uav_msgs::UAVWaypoint> (ns + paramPubTopicSetpoint, 1);

    // Node Ready
    ROS_INFO_STREAM("Priority Manager Node Ready");
}


void Priority_manager::run(){
    
    ros::Rate go(paramNodeRate);
    while (ros::ok()){

        ros::spinOnce();
        go.sleep();
    }
}

void Priority_manager::cbHeifuMsgsHeifuWaypointmsg(const ros::MessageEvent<uav_msgs::UAVWaypoint const>& event, const std::string& topic)
{
    ROS_DEBUG_STREAM("PUBLISHER " << event.getPublisherName());

    const uav_msgs::UAVWaypointConstPtr& msg = event.getMessage();

    ROS_DEBUG_STREAM("TOPIC: " << topic.c_str());
    ROS_DEBUG_STREAM("X message: " << msg->x);
    ROS_DEBUG_STREAM("Priority: " << unsigned(EntryMap[topic].data.priority));

    if(isLockHolder(topic))
    {
        EntryMap[topic].last_read = ros::Time::now();

        pubSetpoint.publish(msg);
    }

}

void Priority_manager::cbGeometryMsgsPoseStampedmsg(const ros::MessageEvent<const geometry_msgs::PoseStamped> &event, const std::string &topic)
{
    ROS_DEBUG_STREAM("PUBLISHER " << event.getPublisherName());

    const geometry_msgs::PoseStampedConstPtr& msg = event.getMessage();

    ROS_DEBUG_STREAM("TOPIC: " << topic.c_str());
    ROS_DEBUG_STREAM("X message: " << msg->pose.position.x);
    ROS_DEBUG_STREAM("Priority: " << unsigned(EntryMap[topic].data.priority));
    if(isLockHolder(topic))
    {
        EntryMap[topic].last_read = ros::Time::now();

        uav_msgs::UAVWaypoint msgPub;

        msgPub.frame = uav_msgs::UAVWaypoint::FRAME_LOCAL_NED;
        msgPub.type  = uav_msgs::UAVWaypoint::TYPE_POSITION;
        msgPub.x = msg->pose.position.x;
        msgPub.y = msg->pose.position.y;
        msgPub.z = msg->pose.position.z;

        tf2::Quaternion q;
        q.setX(msg->pose.orientation.x);
        q.setY(msg->pose.orientation.y);
        q.setZ(msg->pose.orientation.z);
        q.setW(msg->pose.orientation.w);

        q.normalize();

        tf2::Matrix3x3 R(q);
        double roll, pitch, yaw;
        R.getRPY(roll, pitch, yaw);
        msgPub.yaw = static_cast<float>(yaw);

        pubSetpoint.publish(msgPub);
    }
}

void Priority_manager::cbGeometryMsgsTwistStampedmsg(const ros::MessageEvent<geometry_msgs::TwistStamped const>& event, const std::string &topic)
{
    ROS_DEBUG_STREAM("PUBLISHER " << event.getPublisherName());

    const geometry_msgs::TwistStampedConstPtr& msg = event.getMessage();

    ROS_DEBUG_STREAM("TOPIC: " << topic.c_str());
    ROS_DEBUG_STREAM("X message: " << msg->twist.linear.x);
    ROS_DEBUG_STREAM("Priority: " << unsigned(EntryMap[topic].data.priority));
    if(isLockHolder(topic))
    {
        EntryMap[topic].last_read = ros::Time::now();

        uav_msgs::UAVWaypoint msgPub;

        msgPub.frame = uav_msgs::UAVWaypoint::FRAME_LOCAL_NED;
        msgPub.type  = uav_msgs::UAVWaypoint::TYPE_VELOCITY_BYPASS;
        msgPub.x = msg->twist.linear.x;
        msgPub.y = msg->twist.linear.y;
        msgPub.z = msg->twist.linear.z;

        pubSetpoint.publish(msgPub);
    }
}

void Priority_manager::parseTopics(ros::NodeHandle* nh, std::string rosparamName)
{
    XmlRpc::XmlRpcValue filters;
    nh->getParam(rosparamName, filters);

    ROS_ASSERT(filters.getType() == XmlRpc::XmlRpcValue::TypeArray);

    std::string topicName;

    for(uint32_t i = 0; i < filters.size(); ++i) {
        ROS_ASSERT(filters[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(filters[i]["topic"].getType() == XmlRpc::XmlRpcValue::TypeString);

        topicName = static_cast<std::string>(filters[i]["topic"]);

        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = filters[i].begin(); it != filters[i].end(); ++it) {
            if (it->first == "type") {
                ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeString);

                // Check if the message type is known
                M_topic_type::iterator it_type = topic_type_list.find(static_cast<std::string>(it->second));

                if(it_type == topic_type_list.end())
                {
                    ROS_WARN_STREAM("Topic type '" << static_cast<std::string>(it->second) << "' not recognized! Skipping entry.");
                    EntryMap.erase(topicName);
                    break;
                }

                EntryMap[topicName].data.type_id = static_cast<uint8_t>(it_type->second);

                continue;
            }

            if (it->first == "topic") {
                ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeString);
                EntryMap[topicName].data.topic = static_cast<std::string>(it->second);
                continue;
            }

            if (it->first == "priority") {
                ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeInt);

                int auxPriority = static_cast<int>(it->second);
                ROS_ASSERT((auxPriority >= 0) && (auxPriority <= 255));

                EntryMap[topicName].data.priority = static_cast<uint8_t>(auxPriority);
                continue;
            }

            if (it->first == "timeout") {
                ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble);

                double auxTimeout = static_cast<double>(it->second);
                ROS_ASSERT(auxTimeout >= 0.0);

                EntryMap[topicName].data.timeout = auxTimeout;
                EntryMap[topicName].last_read = ros::Time(0,0);
                continue;
            }

        }
    }


    for(EntryMap_::const_iterator it = EntryMap.begin(); it != EntryMap.end(); ++it) {
        ROS_DEBUG_STREAM(std::endl <<
                         it->first << std::endl <<
                         "\t TOPIC:    " << it->second.data.topic              << std::endl <<
                         "\t TYPE ID:  " << unsigned(it->second.data.type_id)  << std::endl <<
                         "\t TIMEOUT:  " << it->second.data.timeout            << std::endl <<
                         "\t PRIORITY: " << unsigned(it->second.data.priority)
                         );
    }
}


void Priority_manager::createSubscribers(ros::NodeHandle *nh)
{
    ROS_INFO_STREAM("Creating subscribers...");

    for(EntryMap_::iterator it = EntryMap.begin(); it != EntryMap.end(); ++it) {

        switch(it->second.data.type_id)
        {
            case uav_msgsHeifuWaypoint:
            {
                std::string topic_name = static_cast<std::string>(it->second.data.topic);
                it->second.subscriber = nh->subscribe<uav_msgs::UAVWaypoint>(topic_name.c_str(), 1, boost::bind(&Priority_manager::cbHeifuMsgsHeifuWaypointmsg, this, _1, topic_name));
                break;
            }
            case geometry_msgsPoseStamped:
            {
                std::string topic_name = static_cast<std::string>(it->second.data.topic);
                it->second.subscriber = nh->subscribe<geometry_msgs::PoseStamped>(topic_name.c_str(), 1, boost::bind(&Priority_manager::cbGeometryMsgsPoseStampedmsg, this, _1, topic_name));
                //it->second.subscriber = nh->subscribe(topic_name.c_str(), 10, &Priority_manager::cbGeometryMsgsPoseStampedmsg, classObj);
                break;
            }
            case geometry_msgsTwistStamped:
            {
                std::string topic_name = static_cast<std::string>(it->second.data.topic);
                it->second.subscriber = nh->subscribe<geometry_msgs::TwistStamped>(topic_name.c_str(), 1, boost::bind(&Priority_manager::cbGeometryMsgsTwistStampedmsg, this, _1, topic_name));
                //it->second.subscriber = nh->subscribe(topic_name.c_str(), 10, &Priority_manager::cbGeometryMsgsTwistStampedmsg, classObj);
                break;
            }
            default:
                break;
        }
    }
}

bool Priority_manager::isLockHolder(std::string topic_name)
{
    // First read
    if (lockHolder == ""){
        lockHolder = topic_name;
        ROS_INFO_STREAM("NEW LOCK HOLDER [FIRST]: " << topic_name);
        return true;
    }

    // Already the holder
    if (lockHolder == topic_name){
        return true;
    }

    // Check priority
    if (EntryMap[topic_name].data.priority > EntryMap[lockHolder].data.priority){
        lockHolder = topic_name;
        ROS_INFO_STREAM("NEW LOCK HOLDER [PRIORITY]: " << topic_name);
        return true;
    }

    // Check timeout
    ros::Time now = ros::Time::now();
    ros::Duration since_last = now - EntryMap[lockHolder].last_read;
    double time_passed = since_last.sec + since_last.nsec*1E-9;

    if (time_passed >= EntryMap[lockHolder].data.timeout){
        lockHolder = topic_name;
        ROS_INFO_STREAM("NEW LOCK HOLDER [TIMEOUT]: " << topic_name);
        return true;
    }

    return false;
}
