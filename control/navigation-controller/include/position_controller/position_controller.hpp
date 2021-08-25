#ifndef POSITION_CONTROLLER_HPP
#define POSITION_CONTROLLER_HPP

#include <ros/ros.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <uav_msgs/UAVWaypoint.h>
#include <tf/transform_datatypes.h>

// Colors
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */

namespace PC{
    class Position_Controller
    {
        public:
            Position_Controller();
            void Run();

            // Variables
            bool posSetpointRec=false;
            bool flagWaypointReceived=false;

            uav_msgs::UAVWaypoint goalPoint;

            uint8_t stage=0;
            uint8_t angleInc;
            uint8_t distanceInc;

        private:
            // ROS Handler
            ros::NodeHandle nh;

            // Parameters
            double paramPosError;
            double paramYawError;

            // Functions
            void cbPosition(const geometry_msgs::PoseStamped &msg);
            bool isAngleWithinRange(double desAngle,double actAngle);
            bool isDistanceWithinRadius(double distance, double radius);
            float calculateWaypointHeading(geometry_msgs::PoseStamped currPos, uav_msgs::UAVWaypoint goalPos);

            // Subscribers
            ros::Subscriber subPosition;

            // Publishers
            ros::Publisher pubLocalPos;
            ros::Publisher pubControlState;

            // Variables
            double WPYaw;
            double maxYawError;

            diagnostic_msgs::KeyValue controlState;
    };
}
#endif 
