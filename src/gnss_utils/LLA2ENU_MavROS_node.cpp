#include "gnss_utils/LLA2ENU_MavROS.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LLA2ENU_MavROS_node");
    LLA2ENU_MavROS LLA2ENU_MavROS_node;
    LLA2ENU_MavROS_node.Run();
    return 0;
}
