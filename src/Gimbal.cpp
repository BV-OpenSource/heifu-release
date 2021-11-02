#include "gimbal/Gimbal.hpp"

using namespace GL;

Gimbal::Gimbal():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Subscribers
    subGimbalGetAxes = n.subscribe(ns + "/gimbal/getAxes", 10, &Gimbal::getAngleGivenAxe, this);
    subGimbalSetAxes = n.subscribe(ns + "/gimbal/setAxes", 10, &Gimbal::setAngleGivenAxe,this);

    // Service Clients
    srvClientparamGetClient = n.serviceClient < mavros_msgs::ParamGet > (ns + "/mavros/param/get");

    // Publishers
    pubMountControl = n.advertise<mavros_msgs::MountControl>(ns + "/mavros/mount_control/command", 1);

    // Parameters
    n.param<double>("paramControlHz",       paramControlHz,  10.0);
    n.param<double>("paramLimitRollMax",    paramLimitRollMax,  35.0);
    n.param<double>("paramLimitRollMin",    paramLimitRollMin,  -35.0);
    n.param<double>("paramLimitPitchMax",   paramLimitPitchMax, 90.0);
    n.param<double>("paramLimitPitchMin",   paramLimitPitchMin, -90.0);
    n.param<double>("paramLimitYawMax",     paramLimitYawMax,  180.0);
    n.param<double>("paramLimitYawMin",     paramLimitYawMin,  -180.0);

    // Timer
    angleTimer = n.createTimer(ros::Duration(1/paramControlHz), &Gimbal::setAngleMavros, this, false, true);

    // Variables
    rollAngle  = std::numeric_limits<double>::quiet_NaN(); // ((paramLimitRollMax - paramLimitRollMin) / 2) + paramLimitRollMin;
    pitchAngle = std::numeric_limits<double>::quiet_NaN(); // ((paramLimitPitchMax - paramLimitPitchMin) / 2) + paramLimitPitchMin;
    yawAngle   = std::numeric_limits<double>::quiet_NaN(); // ((paramLimitYawMax - paramLimitYawMin) / 2) + paramLimitYawMin;

    oldYawAngle = 0.0;
    oldRollAngle = 0.0;
    oldPitchAngle = 0.0;

    angleXRead = false;
    angleYRead = false;
    angleZRead = false;

    pubGimbalControl.mode = pubGimbalControl.MAV_MOUNT_MODE_MAVLINK_TARGETING;

    msgGetGimbalAxes.axe = "all";

    ROS_INFO("Gimbal Node Ready");
    ROS_INFO("The angles values must be in the aceptables intervals.");
    ROS_INFO("%f<X<%f || %f<Y<%f || %f<Z<%f",
             paramLimitRollMin, paramLimitRollMax,
             paramLimitPitchMin, paramLimitPitchMax,
             paramLimitYawMin,paramLimitYawMax);
}

void Gimbal::run(){

    ros::Rate go(0.2);

    while (ros::ok()){
        if(angleXRead && angleYRead && angleZRead){
            go = ros::Rate(100);
        }else{
            this->getAngleGivenAxe(msgGetGimbalAxes);
        }

        ros::spinOnce();
        go.sleep();
    }
}

void Gimbal::setAngleMavros(const ros::TimerEvent&){
    if( (fabs(oldRollAngle - rollAngle) > DBL_EPSILON && angleXRead) ||
            (fabs(oldYawAngle - yawAngle) > DBL_EPSILON && angleZRead) ||
            (fabs(oldPitchAngle - pitchAngle) > DBL_EPSILON && angleYRead)){
        oldYawAngle = yawAngle;
        oldRollAngle = rollAngle;
        oldPitchAngle = pitchAngle;
        ROS_INFO("Roll-> %f || Pitch-> %f || Yaw-> %f", rollAngle, pitchAngle, yawAngle);
        pubGimbalControl.roll = static_cast<float>(rollAngle * 100.0);
        pubGimbalControl.pitch = static_cast<float>(pitchAngle * 100.0);
        pubGimbalControl.yaw = static_cast<float>(yawAngle * 100.0);
        pubMountControl.publish(pubGimbalControl);
    }
}

void Gimbal::setAngleGivenAxe(const gimbal::setGimbalAxes &msg){
    if( msg.roll != 0.0 && angleXRead){
        rollAngle += msg.roll;
        isBeyondAngleMax(rollAngle, paramLimitRollMax);
        isBeyondAngleMin(rollAngle, paramLimitRollMin);
    }

    if( msg.pitch != 0.0 && angleYRead){
        pitchAngle += msg.pitch;
        isBeyondAngleMax(pitchAngle, paramLimitPitchMax);
        isBeyondAngleMin(pitchAngle, paramLimitPitchMin);
    }

    if( msg.yaw != 0.0 && angleZRead){
        yawAngle += msg.yaw;
        isBeyondAngleMax(yawAngle, paramLimitYawMax);
        isBeyondAngleMin(yawAngle, paramLimitYawMin);
    }
}

void Gimbal::getAngleGivenAxe(const gimbal::getGimbalAxes &msg){

    std::string axes = msg.axe;

    if (axes.find("all") != std::string::npos){
        paramGetMsg.request.param_id = "MNT_NEUTRAL_X";
        if(!angleXRead){
            if (srvClientparamGetClient.call(paramGetMsg) && paramGetMsg.response.success){
                oldRollAngle = rollAngle = paramGetMsg.response.value.real;
                ROS_INFO("Roll with value %f",rollAngle);
                angleXRead = true;
            }
        }

        if(!angleYRead){
            paramGetMsg.request.param_id = "MNT_NEUTRAL_Y";
            if (srvClientparamGetClient.call(paramGetMsg) && paramGetMsg.response.success){
                oldPitchAngle = pitchAngle = paramGetMsg.response.value.real;
                ROS_INFO("Pitch with value %f",pitchAngle);
                angleYRead = true;
            }
        }

        if(!angleZRead){
            paramGetMsg.request.param_id = "MNT_NEUTRAL_Z";
            if (srvClientparamGetClient.call(paramGetMsg) && paramGetMsg.response.success){
                oldYawAngle = yawAngle = paramGetMsg.response.value.real;
                ROS_INFO("Yaw with value %f",yawAngle);
                angleZRead = true;
            }
        }
    }
    else {
        if (axes.find('x') != std::string::npos){
            paramGetMsg.request.param_id = "MNT_NEUTRAL_X";
            if (srvClientparamGetClient.call(paramGetMsg) && paramGetMsg.response.success){
                rollAngle = paramGetMsg.response.value.real;
                ROS_INFO("Roll with value %f",rollAngle);
            }
        }
        if (axes.find('y') != std::string::npos) {
            paramGetMsg.request.param_id = "MNT_NEUTRAL_Y";
            if (srvClientparamGetClient.call(paramGetMsg) && paramGetMsg.response.success){
                pitchAngle = paramGetMsg.response.value.real;
                ROS_INFO("Pitch with value %f",pitchAngle);
            }
        }
        if (axes.find('z') != std::string::npos) {
            paramGetMsg.request.param_id = "MNT_NEUTRAL_Z";
            if (srvClientparamGetClient.call(paramGetMsg) && paramGetMsg.response.success){
                yawAngle = paramGetMsg.response.value.real;
                ROS_INFO("Yaw with value %f",yawAngle);
            }
        }
    }
}

bool Gimbal::isBeyondAngleMax(double &value, double max){
    bool output = value > max;
    if(output){
        value = max;
    }
    return output;
}

bool Gimbal::isBeyondAngleMin(double &value, double min){
    bool output = value < min;
    if(output){
        value = min;
    }
    return output;
}
