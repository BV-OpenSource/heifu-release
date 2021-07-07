#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>

#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/ParamPull.h>

#include <std_msgs/Empty.h>
#include "gimbal/setGimbalAxes.h"
#include "gimbal/getGimbalAxes.h"

namespace GL{

    class Gimbal {

        public:

            Gimbal ();
            virtual ~Gimbal (){}

            void run();

        private:

            // Node handles
            ros::NodeHandle n;

            // ROS Subscribers
            ros::Subscriber subGimbalGetAxes;
            ros::Subscriber subGimbalSetAxes;

            // ROS Service Client
            ros::ServiceClient srvClientparamSetClient;
            ros::ServiceClient srvClientparamGetClient;
            ros::ServiceClient srvGetMavrosParam;

            // Message
            mavros_msgs::ParamSet paramSetMsg;
            mavros_msgs::ParamGet paramGetMsg;
            mavros_msgs::ParamPull paramPull;

            // Parameters
            double paramControlHz;
            double paramLimitRollMax;
            double paramLimitRollMin;
            double paramLimitPitchMax;
            double paramLimitPitchMin;
            double paramLimitYawMax;
            double paramLimitYawMin;

            // Variables
            double yawAngle;
            double rollAngle;
            double pitchAngle;
            double oldYawAngle;
            double oldRollAngle;
            double oldPitchAngle;

            bool angleXRead;
            bool angleYRead;
            bool angleZRead;

            ros::Timer angleTimer;

            gimbal::getGimbalAxes msgGetGimbalAxes;

            // Functions
            bool isBeyondAngleMax(double value, double max);
            bool isBeyondAngleMin(double value, double max);
            void setAngleMavros(const ros::TimerEvent &);
            void getAngleGivenAxe(const gimbal::getGimbalAxes &msg);
            bool callServiceMavros(const std::string paramId, double initialValue, double* currentAngle);
            void setAngleGivenAxe(const gimbal::setGimbalAxes &msg);

    };
}



#endif

