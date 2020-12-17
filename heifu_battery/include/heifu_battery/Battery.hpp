#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <ros/ros.h>
#include <mavros_msgs/BatteryStatus.h>
#include <sensor_msgs/BatteryState.h>

namespace BT
{
    class Battery
    {
    public:

        Battery ();
        virtual ~Battery (){};

        void run();

    private:
        ros::NodeHandle n;

        // Subscribers
        ros::Subscriber subBatt;

        // Publishers
        ros::Publisher pubBatt;

        // Functions
        void cbBatt(const sensor_msgs::BatteryState& msg);
    };

}

#endif
