#ifndef MAVROS_COMMANDS_HPP
#define MAVROS_COMMANDS_HPP

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>

namespace MC
{
    class Mavros_commands
    {
    public:

        Mavros_commands();
        virtual ~Mavros_commands (){};

        void run();

    private:
        ros::NodeHandle n;

        // Subscribers
        ros::Subscriber subArm;
        ros::Subscriber subAuto;
        ros::Subscriber subBrake;
        ros::Subscriber subDisarm;
        ros::Subscriber subGuided;
        ros::Subscriber subLand;
        ros::Subscriber subLoiter;
        ros::Subscriber subMissionStart;
        ros::Subscriber subMissionStop;
        ros::Subscriber subReturnHome;
        ros::Subscriber subState;
        ros::Subscriber subTakeOff;
        ros::Subscriber subUAVPose;

        // Services Clients
        ros::ServiceClient srvClientArming;
        ros::ServiceClient srvClientSetMode;
        ros::ServiceClient srvClientTakeOff;

        // Publishers
        ros::Publisher pubRequestReached;
        ros::Publisher pubLandDiagnostic;
        ros::Publisher pubTakeoffDiagnostic;

        // Parameters
        double paramNodeRate;
        double paramTakeOffAltitude;
        double paramTakeOffThreshold;

        std::string paramSubTopicArm;
        std::string paramSubTopicAutoMode;
        std::string paramSubTopicBrakeMode;
        std::string paramSubTopicDisarm;
        std::string paramSubTopicGuidedMode;
        std::string paramSubTopicLand;
        std::string paramSubTopicLoiter;
        std::string paramSubTopicMissionStart;
        std::string paramSubTopicMissionStop;
        std::string paramSubTopicRTL;
        std::string paramSubTopicTakeoff;
        std::string paramSubTopicUAVPose;

        std::string paramPubTopicDiagTakeoff;
        std::string paramPubTopicDiagLand;

        // Variables
        bool    ongoingTakeOff;
        bool    landingOrRTL;
        bool    onMission;
        bool    modeAuto;
        double  lastAltitude;
        double  currentAltitude;
        double  desiredAltitude;
        double  takeOffAltitude;
        uint8_t currentState;
        uint8_t takeoffWatchdog;

        std_msgs::Empty emptyMsg;

        // Functions
        void cbArm(const std_msgs::EmptyConstPtr);
        void cbAuto(const std_msgs::EmptyConstPtr);
        void cbBrake(const std_msgs::EmptyConstPtr);
        void cbDisarm(const std_msgs::EmptyConstPtr);
        void cbGuided(const std_msgs::EmptyConstPtr);
        void cbMissionStart(const std_msgs::EmptyConstPtr);
        void cbMissionStop(const std_msgs::EmptyConstPtr);
        void cbLand(const std_msgs::EmptyConstPtr);
        void cbLoiter(const std_msgs::EmptyConstPtr);
        void cbRTL(const std_msgs::EmptyConstPtr);
        void cbState(const mavros_msgs::StateConstPtr& msg);
        void cbTakeOff(const std_msgs::UInt8 msg);
        void cbUAVPose(const geometry_msgs::PoseStampedConstPtr& msg);

        void changeToAutoMode();
        void changeToGuidedMode();
        bool requestArm(bool arm);
        bool requestTakeoff();
        bool setMode(std::string mode);
    };

}

#endif
