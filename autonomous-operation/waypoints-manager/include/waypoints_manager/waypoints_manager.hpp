#ifndef WAYPOINTS_MANAGER_HPP
#define WAYPOINTS_MANAGER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <uav_msgs/UAVWaypoint.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/WaypointList.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>

// Commands
#define WAYPOINT            16
#define LOITER              17
#define RETURNTOLAUNCH      20
#define LAND                21
#define TAKEOFF             22
#define VTOL_TAKEOFF        84
#define WAIT_FOR_HEADING    115
#define SPEED_CHANGE        178
#define TAKEPHOTO           206
#define VTOL_TRANSITION     3000

// Frames
#define FRAME_GLOBAL            0
#define FRAME_LOCAL_NED         1
#define FRAME_MISSION           2
#define FRAME_GLOBAL_REL_ALT    3
#define FRAME_LOCAL_ENU         4

// Colors
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

// Vehicles
#define HEXAROTOR   0
#define HVTOL       1

class Waypoints_Manager{
    public:
        Waypoints_Manager();
        ~Waypoints_Manager();
        void Run();

    private:
        // ROS Handler
        ros::NodeHandle nh;
        std::string ns;

        // Parameters
        double paramNodeRate;

        int paramVehicleType;
        int paramAutopilotType;

        std::string paramSubStartPlan;
        std::string paramSubSetpointPoint;
        std::string paramSubReachedPosition;
        std::string paramSubConvertedGoalPoint;

        std::string paramPubRTL;
        std::string paramPubToFW;
        std::string paramPubToMC;
        std::string paramPubLand;
        std::string paramPubTakeoff;
        std::string paramPubPlannersStart;
        std::string paramPubPoseCommandLLA;
        std::string paramPubGoalPointOutput;
        std::string paramPubPlannersSetpoint;
        std::string paramPubLandingPadPosition;
        std::string paramPubLandingEnableTopic;

        // Variables
        sensor_msgs::NavSatFix landingPad;
        sensor_msgs::NavSatFix HomePosition;
        sensor_msgs::NavSatFix WaypointHome;
        sensor_msgs::NavSatFix currGlobalPosition;

        mavros_msgs::WaypointList vectorWaypoints;

        std_msgs::Empty empty;

        bool endLanding;
        bool takeoffSent;
        bool plannersIsRunning;
        bool lastPositionIsLand;

        float loiterRadius;
        float waypointRadius;
        float missionVelocity;

        uint waypointCommand;

        size_t vectorWaypointsSize = 0;

        uint16_t currentDesiredPosition = 0;

        // Subscribers
        ros::Subscriber subPlanners;
        ros::Subscriber subStartPlan;
        ros::Subscriber subHomePosition;
        ros::Subscriber subGlobalPosition;
        ros::Subscriber subPlannersReached;
        ros::Subscriber subWaypointsMavros;
        ros::Subscriber subReachedPosition;
        ros::Subscriber subSetpointCoordinates;
        ros::Subscriber subConvertedCoordinates;

        // Publishers
        ros::Publisher pubRTL;
        ros::Publisher pubToFW;
        ros::Publisher pubToMC;
        ros::Publisher pubLand;
        ros::Publisher pubTakeoff;
        ros::Publisher pubGoalENUPoint;
        ros::Publisher pubEnableLanding;
        ros::Publisher pubPlannersStart;
        ros::Publisher pubPoseCommandLLA;
        ros::Publisher pubPlannersSetpoint;
        ros::Publisher pubLandingPadPosition;

        // Functions
        void cbStartPlan(const std_msgs::Empty);
        void cbPlannersStart(const std_msgs::Empty);
        void cbPlannersFinish(const std_msgs::Empty);
        void cbReachedPosition(const std_msgs::Empty);
        void cbHomePosition(const geographic_msgs::GeoPointStamped& msg);
        void cbWaypointsList(const mavros_msgs::WaypointListConstPtr& msg);
        void cbGlobalPosition(const sensor_msgs::NavSatFix::ConstPtr msg);
        void cbSetpointCoordinates(const geographic_msgs::GeoPointConstPtr msg);
        void cbConvertedCoordinates(const geometry_msgs::PointStampedConstPtr msg);
        void pubDesiredPosition(mavros_msgs::WaypointList plan, size_t index);
        void pubWaypoint(const mavros_msgs::Waypoint::_frame_type frame, const double x, const double y, const double z);
};

#endif
