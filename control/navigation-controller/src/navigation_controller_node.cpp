#include "navigation_controller/navigation_controller.hpp"

using namespace NC;

int main(int argc, char **argv){
    ros::init(argc, argv, "navigation_controller_node");
    Navigation_Controller navigationController;
    navigationController.Run();
    return 0;
}
