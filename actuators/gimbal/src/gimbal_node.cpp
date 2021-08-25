#include "gimbal/Gimbal.hpp"

using namespace GL;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_node");
    GL::Gimbal gl;
    gl.run();
    return 0;
}

