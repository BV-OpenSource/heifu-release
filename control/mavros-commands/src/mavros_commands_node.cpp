#include "mavros_commands/Mavros_commands.hpp"

using namespace MC;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_commands_node");
    MC::Mavros_commands mavrosCommands;
    mavrosCommands.run();
    return 0;
}
