#include "gimbal/Gimbal.hpp"

using namespace GL;

Gimbal::Gimbal():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Subscribers
    subGimbalGetAxes = n.subscribe(ns + "/gimbal/getAxes", 10, &Gimbal::getAngleGivenAxe, this);
    subGimbalSetAxes = n.subscribe(ns + "/gimbal/setAxes", 10, &Gimbal::setAngleGivenAxe,this);

    // Service Clients
    srvClientparamGetClient = n.serviceClient < mavros_msgs::ParamGet > (ns + "/mavros/param/get");
    srvClientparamSetClient = n.serviceClient < mavros_msgs::ParamSet > (ns + "/mavros/param/set");
    srvGetMavrosParam = n.serviceClient < mavros_msgs::ParamSet > (ns + "/mavros/param/pull");

    // Parameters
    n.param<double>("paramControlHz",       paramControlHz,  1.0);
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
    if( fabs(oldRollAngle - rollAngle) > DBL_EPSILON && angleXRead){
        callServiceMavros("MNT_NEUTRAL_X", oldRollAngle, &rollAngle);
        oldRollAngle = rollAngle;
        ROS_INFO("Roll-> %f || Pitch-> %f || Yaw-> %f", rollAngle, pitchAngle, yawAngle);
    }

    if( fabs(oldPitchAngle - pitchAngle) > DBL_EPSILON && angleYRead){
        callServiceMavros("MNT_NEUTRAL_Y", oldPitchAngle, &pitchAngle);
        oldPitchAngle = pitchAngle;
        ROS_INFO("Roll-> %f || Pitch-> %f || Yaw-> %f", rollAngle, pitchAngle, yawAngle);
    }

    if( fabs(oldYawAngle - yawAngle) > DBL_EPSILON && angleZRead){
        callServiceMavros("MNT_NEUTRAL_Z", oldYawAngle, &yawAngle);
        oldYawAngle = yawAngle;
    }
}

void Gimbal::setAngleGivenAxe(const gimbal::setGimbalAxes &msg){
    if( msg.roll != 0.0 && angleXRead){
        rollAngle += msg.roll;
        if(isBeyondAngleMax(rollAngle, paramLimitRollMax) || isBeyondAngleMin(rollAngle, paramLimitRollMin)){
            rollAngle -= msg.roll;
        }
    }

    if( msg.pitch != 0.0 && angleYRead){
        pitchAngle += msg.pitch;
        if(isBeyondAngleMax(pitchAngle, paramLimitPitchMax) || isBeyondAngleMin(pitchAngle, paramLimitPitchMin)){
            pitchAngle -= msg.pitch;
        }
    }

    if( msg.yaw != 0.0 && angleZRead){
        yawAngle += msg.yaw;
        if(isBeyondAngleMax(yawAngle, paramLimitYawMax) || isBeyondAngleMin(yawAngle, paramLimitYawMin)){
            yawAngle -= msg.yaw;
        }
    }
}

bool Gimbal::callServiceMavros(const std::string paramId, double initialValue, double* currentAngle){
    paramSetMsg.request.param_id = paramId;
    paramSetMsg.request.value.integer = 0;
    paramSetMsg.request.value.real = *currentAngle;

    if (srvClientparamSetClient.call(paramSetMsg) && paramSetMsg.response.success){
        //ROS_INFO("%s value set with sucess with value %f",axes.c_str(),paramSetMsg.response.value.real);
        return true;
    }
    else{
        *currentAngle = initialValue;
        ROS_ERROR("Something is wrong with the conection.");
        paramPull.request.force_pull = true;
        //TRY GET MAVROS PARAM AGAIN
        if (srvGetMavrosParam.call(paramPull) && paramPull.response.success){
            ROS_INFO("Mavros Param Received.");
        }
        else{
            ROS_ERROR("Erro getting Mavros Param.");
        }
        return false;
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

bool Gimbal::isBeyondAngleMax(double value, double max){
    return value > max;
}

bool Gimbal::isBeyondAngleMin(double value, double min){
    return value < min;
}
