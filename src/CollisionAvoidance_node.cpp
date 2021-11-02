#include "CollisionAvoidance/CollisionAvoidance.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CollisionAvoidance_node");
    CollisionAvoidance CollisionAvoidance_node;
    CollisionAvoidance_node.Run();
    return 0;
}
