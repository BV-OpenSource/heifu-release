#include "gnss_utils/LLA2ENU_MavROS.hpp"

LLA2ENU_MavROS::LLA2ENU_MavROS():nh("~")
{
    std::string ns = ros::this_node::getNamespace();

    std::string home_topic;
    std::string converted;
    std::string goal_coord;
    std::string homeConvertedTopic;

    nh.param<std::string>("paramGoalCoord", goal_coord,             "/gnss_utils/goal_coordinates");
    nh.param<std::string>("paramHomeTopic", home_topic,             "/mavros/global_position/gp_origin");
    nh.param<std::string>("paramConverted", converted,              "/gnss_utils/converted_coordinates");
    nh.param<std::string>("paramHomeTopic", homeConvertedTopic,     "/gnss_utils/home_LLA");

    converted_pub       = nh.advertise<geometry_msgs::PointStamped>(ns + converted, 10);
    homeConvertedPub    = nh.advertise<geographic_msgs::GeoPointStamped>(ns + homeConvertedTopic, 10);

    goal_sub = nh.subscribe(ns + goal_coord, 10, &LLA2ENU_MavROS::cbGoal, this);
    home_sub = nh.subscribe(ns + home_topic, 10, &LLA2ENU_MavROS::cbHome, this);

    srv = nh.serviceClient<std_srvs::Trigger>(ns + "/mavros/home_position/req_update");
}

void LLA2ENU_MavROS::Run(){
    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        if(!home_defined){
            std_srvs::Trigger command;
            srv.call(command.request, command.response);
        }else{
            loop_rate = ros::Rate(100);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void LLA2ENU_MavROS::cbGoal(const geographic_msgs::GeoPointStamped &msg){
    geometry_msgs::PointStamped converted;
    if(home_defined){
        GNSS_utils::LLAtoENU(msg.position.latitude, msg.position.longitude, msg.position.altitude,
                             home.latitude, home.longitude, home.altitude,
                             converted.point.x, converted.point.y, converted.point.z);
        converted.header = msg.header;
        converted_pub.publish(converted);
    }else {
        ROS_WARN("Home not defined");
    }
}

void LLA2ENU_MavROS::cbHome(const geographic_msgs::GeoPointStamped &msg){
    GNSS_utils::ECEFtoLLA(msg.position.latitude, msg.position.longitude, msg.position.altitude,
                          home.latitude, home.longitude, home.altitude);

    geographic_msgs::GeoPointStamped geoPoint;
    geoPoint.header.stamp = ros::Time::now();
    geoPoint.position = home;
    homeConvertedPub.publish(geoPoint);

    home_defined = true;
}
