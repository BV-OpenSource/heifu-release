#ifndef LOITER_CONTROLLER_HPP
#define LOITER_CONTROLLER_HPP

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <hvtol_msgs/HVTOLWaypoint.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>

// Colors
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */

namespace LC{
    class Loiter_Controller
    {
        public:
            Loiter_Controller();
            void Run();

            //Variables
            bool loiterSetpointRec=false;
            bool flagWaypointReceived=false;
            bool canIncrement=false;
            hvtol_msgs::HVTOLWaypoint goalPoint;
            float x_check=0;
            float y_check=0;
            uint8_t loiter_point_check=0;
            uint8_t counter;
        private:
            // ROS Handler
            ros::NodeHandle nh;

            // Parameters
            double paramLoiterError;
            double paramNodeRate;
            int paramNumberLaps;

            // Functions
            void cbPosition(const geometry_msgs::PoseStamped &msg);
            float getDistance2D(double x1, double y1, double x2, double y2);
            float getDistance3D(double x1, double y1, double z1, double x2, double y2, double z2);
            bool checkLoiterStart(float currX, float currY, float currZ);

            // Subscribers;
            ros::Subscriber subPosition;

            // Publishers;
            ros::Publisher pubLocalPos;
    };
}
#endif 
