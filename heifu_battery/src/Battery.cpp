#include "heifu_battery/Battery.hpp"

using namespace BT;

Battery::Battery():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Subscribers
    subBatt           = n.subscribe(ns + "/mavros/battery", 1, &Battery::cbBatt, this);

    // Publishers
    pubBatt          = n.advertise < sensor_msgs::BatteryState > (ns + "/battery", 1);
}


void Battery::run(){
    ros::Rate go(1);
    while (ros::ok()){
        ros::spinOnce();
        go.sleep();
    }
}

void Battery::cbBatt(const sensor_msgs::BatteryState& msg){
    pubBatt.publish(msg);
}