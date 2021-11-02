#include "RRT/RRT.hpp"

using namespace RRT3D;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_node");

    RRT RRT_node;
    RRT_node.Setup();
    RRT_node.Run();

    return 0;
}
