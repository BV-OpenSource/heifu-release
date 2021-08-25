#ifndef NAVIGATION_CONTROLLER_HPP
#define NAVIGATION_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <uav_msgs/UAVWaypoint.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>

#include "velocity_controller/velocity_controller.hpp"
#include "position_controller/position_controller.hpp"

#define RESET_VELOCITY_BYPASS_TIMER 0.2

// Colors
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */

namespace NC{

    class Navigation_Controller
    {
        public:
            Navigation_Controller();
            ~Navigation_Controller();
            void Run();

            VC::Velocity_Controller* vc = nullptr;
            PC::Position_Controller* pc = nullptr;

            // Functions
            //void cbSetpoint(const geometry_msgs::PointStamped &msg);
            void cbSetpoint(const uav_msgs::UAVWaypoint &msg);
            void cbState(const mavros_msgs::State::ConstPtr msg);
            void cbExtendedState(const mavros_msgs::ExtendedState::ConstPtr msg);
            void change_parameter(std::string string_rec, double value);
            double get_parameter(std::string string_rec);

            // Variables
            geometry_msgs::PoseStamped localPosition;
            bool flagWaypointReceived;
            bool flagBypass = false;
            bool waypointReachedSkip;
            double cbSetpointTime;

        private:
            // ROS Handler
            ros::NodeHandle nh;

            // Parameters
            double paramNodeRate;
            std::string paramSubTopicSetpoint;
            std::string paramPubTopicWPReached;

            // Variables
            geometry_msgs::PointStamped goalPoint;
            mavros_msgs::ExtendedState  currentExtendedState;
            mavros_msgs::PositionTarget velBypass;
            mavros_msgs::State currentState;
            std_msgs::Empty msgEmpty;


            // Subscribers;
            ros::Subscriber subExtendedState;
            ros::Subscriber subSetpoint;
            ros::Subscriber subState;

            // Publishers;
            ros::Publisher pubOffboard;
            ros::Publisher pubPosHold;
            ros::Publisher pubVelocityBypass;
            ros::Publisher pubWaypointReached;

            // Services;
            ros::ServiceClient srvClientGetParam;
            ros::ServiceClient srvClientSetParam;

    };
    
}
#endif
