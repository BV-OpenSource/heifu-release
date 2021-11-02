#ifndef STATUS_DIAGNOSTIC_HPP
#define STATUS_DIAGNOSTIC_HPP

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/Int8.h>

namespace SD
{
    class Status_diagnostic
    {
    public:

        Status_diagnostic ();
        virtual ~Status_diagnostic (){}

        void run();

    private:
        ros::NodeHandle n;

        // Subscribers
        ros::Subscriber subDiagnostic;

        // Publishers
        ros::Publisher pubGpsFixState;
        ros::Publisher pubVehicleType;
        ros::Publisher pubAutoPilotType;

        // Parameters
        double paramNodeRate;
        std::string paramSubTopicDiagnostic;
        std::string paramPubTopicGPSFixType;
        std::string paramPubTopicVehicleType;
        std::string paramPubTopicAutoPilotType;

        // Variables
        std_msgs::Int8 pubIntData;
        std::vector<std::string> gpsFixTypeName;

        std::vector<std::string> msgString;
        std::vector<std::string> keyString;
        std::vector<std::string> nameString;

        // Functions
        void cbDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr msg);
    };

}

#endif
