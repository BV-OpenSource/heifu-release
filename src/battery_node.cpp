#include "heifu_battery/Battery.hpp"

using namespace BT;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_node");
    BT::Battery battery;
    battery.run();
    return 0;
}
