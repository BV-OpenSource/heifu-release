#ifndef VELOCITY_CONTROLLER_HPP
#define VELOCITY_CONTROLLER_HPP

#include <ros/ros.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/PositionTarget.h>
#include <uav_msgs/UAVWaypoint.h>

// Colors
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */

namespace VC{
    class Velocity_Controller
    {
        public:
            Velocity_Controller();
            void Run();

            // Variables
            bool velSetpointRec = false;

            uav_msgs::UAVWaypoint goalPoint;
            geometry_msgs::TwistStamped velocity;
            geometry_msgs::PoseStamped localPosition;

            uint8_t stage;
            uint8_t angleInc;
            uint8_t distanceInc;

            // Functions
            void changeMaxSpeed(float maxSpeed);

        private:
            // ROS Handler
            ros::NodeHandle nh;

            // Parameters
            double paramPosError;
            double paramYawError;
            double paramMaxVelocity;
            double paramSmoothFactor;
            double paramReachDistance;
            double paramForceLinearSpeed;
            double paramMaxAngularVelocity;

            std::string paramPubTopicVelocity;

            // Functions
            void calculateVelocitySetpoints();
            void calculateYawRate(double desAngle);
            void cbPosition(const geometry_msgs::PoseStamped &msg);

            bool isAngleWithinRange(double desAngle,double actAngle);

            double calculateDiffAngle(double desAngle, double actAngle);
            double calculateWaypointHeading(geometry_msgs::PoseStamped currPos, uav_msgs::UAVWaypoint goalPos);
            bool isDistanceWithinRadius(double distance, double radius);

            // Subscribers;
            ros::Subscriber subPosition;

            // Publishers;
            ros::Publisher pubVelocity;
            ros::Publisher pubControlState;

            // Variables
            double yaw;
            double WPYaw;
            double errorNorm;
            double maxYawError;
            double maxLinearVelocity;
            double maxAngularVelocity;

            double deg2rad = M_PI/180.0;

            diagnostic_msgs::KeyValue controlState;
    };
}
#endif // VELOCITY_CONTROLLER_HPP
