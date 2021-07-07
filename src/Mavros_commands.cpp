#include "mavros_commands/Mavros_commands.hpp"

using namespace MC;

Mavros_commands::Mavros_commands():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Parameters
    n.param<double>("paramTakeOffAltitude",  paramTakeOffAltitude,  10.0);
    n.param<double>("paramTakeOffThreshold", paramTakeOffThreshold, 0.3);
    n.param<double>("paramNodeRate",         paramNodeRate,         100.0);

    n.param<std::string>("paramSubTopicAutoMode",     paramSubTopicAutoMode,     "/mode_auto");
    n.param<std::string>("paramSubTopicBrakeMode",    paramSubTopicBrakeMode,    "/mode_brake");
    n.param<std::string>("paramSubTopicGuidedMode",   paramSubTopicGuidedMode,   "/mode_guided");
    n.param<std::string>("paramSubTopicTakeoff",      paramSubTopicTakeoff,      "/takeoff");
    n.param<std::string>("paramSubTopicLand",         paramSubTopicLand,         "/land");
    n.param<std::string>("paramSubTopicLoiter",       paramSubTopicLoiter,       "/loiter");
    n.param<std::string>("paramSubTopicRTL",          paramSubTopicRTL,          "/rtl");
    n.param<std::string>("paramSubTopicDisarm",       paramSubTopicDisarm,       "/disarm");
    n.param<std::string>("paramSubTopicArm",          paramSubTopicArm,          "/arm");
    n.param<std::string>("paramSubTopicStartMission", paramSubTopicMissionStart, "/mission/start");
    n.param<std::string>("paramSubTopicStopMission",  paramSubTopicMissionStop,  "/mission/stop");
    n.param<std::string>("paramSubTopicUAVPose",      paramSubTopicUAVPose,      "/mavros/local_position/pose");

    n.param<std::string>("paramPubTopicDiagTakeoff",  paramPubTopicDiagTakeoff, "/diagnostic/takeoff");
    n.param<std::string>("paramPubTopicDiagLand",     paramPubTopicDiagLand,    "/diagnostic/land");

    // Subscribers
    subArm           = n.subscribe(ns + paramSubTopicArm,          10, &Mavros_commands::cbArm,          this);
    subAuto          = n.subscribe(ns + paramSubTopicAutoMode,     10, &Mavros_commands::cbAuto,         this);
    subBrake         = n.subscribe(ns + paramSubTopicBrakeMode,    10, &Mavros_commands::cbBrake,        this);
    subDisarm        = n.subscribe(ns + paramSubTopicDisarm,       10, &Mavros_commands::cbDisarm,       this);
    subGuided        = n.subscribe(ns + paramSubTopicGuidedMode,   10, &Mavros_commands::cbGuided,       this);
    subLand          = n.subscribe(ns + paramSubTopicLand,         10, &Mavros_commands::cbLand,         this);
    subLoiter        = n.subscribe(ns + paramSubTopicLoiter,       10, &Mavros_commands::cbLoiter,       this);
    subMissionStart  = n.subscribe(ns + paramSubTopicMissionStart, 10, &Mavros_commands::cbMissionStart, this);
    subMissionStop   = n.subscribe(ns + paramSubTopicMissionStop,  10, &Mavros_commands::cbMissionStop,  this);
    subUAVPose       = n.subscribe(ns + paramSubTopicUAVPose,      1,  &Mavros_commands::cbUAVPose,      this);
    subState         = n.subscribe(ns + "/mavros/state",           10, &Mavros_commands::cbState,        this);
    subReturnHome    = n.subscribe(ns + paramSubTopicRTL,          10, &Mavros_commands::cbRTL,          this);
    subTakeOff       = n.subscribe(ns + paramSubTopicTakeoff,      10, &Mavros_commands::cbTakeOff,      this);

    // Service Clients
    srvClientArming    = n.serviceClient < mavros_msgs::CommandBool > (ns + "/mavros/cmd/arming");
    srvClientSetMode   = n.serviceClient < mavros_msgs::SetMode > (ns + "/mavros/set_mode");
    srvClientTakeOff   = n.serviceClient < mavros_msgs::CommandTOL > (ns + "/mavros/cmd/takeoff");

    // Publishers
    pubLandDiagnostic    = n.advertise <std_msgs::Bool> (ns + paramPubTopicDiagLand,    1);
    pubRequestReached    = n.advertise <std_msgs::Empty> (ns + "/waypoint_reached",      1);
    pubTakeoffDiagnostic = n.advertise <std_msgs::Bool> (ns + paramPubTopicDiagTakeoff, 1);

    // Variables
    ongoingTakeOff  = false;
    landingOrRTL = false;
    currentAltitude = desiredAltitude = lastAltitude = 0.0;
    takeoffWatchdog = 0;
    onMission = false;
    modeAuto  = false;

    // Node Ready
    ROS_INFO_STREAM("Mavros Commands Node Ready");
}

void Mavros_commands::run(){
    ros::Rate go(paramNodeRate);
    while (ros::ok()){
        if(ongoingTakeOff){
            ROS_INFO_THROTTLE(1.0, "UAV Doing TakeOff!");
            ongoingTakeOff = (currentAltitude < desiredAltitude - paramTakeOffThreshold);

            ROS_INFO_STREAM_COND(!ongoingTakeOff, "Takeoff Position Reached!");
            if(!ongoingTakeOff){
                pubRequestReached.publish(emptyMsg);
            }
        }

        if (modeAuto){
            changeToAutoMode();
        }
        
        ros::spinOnce();
        go.sleep();
    }
}

void Mavros_commands::cbTakeOff(const std_msgs::UInt8 msg){
    std_msgs::Bool takeoffResult;

    takeoffResult.data = setMode("GUIDED");
    takeOffAltitude = msg.data > paramTakeOffAltitude ? msg.data : paramTakeOffAltitude;
    ROS_INFO_STREAM(takeOffAltitude << " m TAKEOFF");

    if (takeoffResult.data){
        takeoffResult.data = requestArm(true);
        if (takeoffResult.data){
            ongoingTakeOff = takeoffResult.data = requestTakeoff();
            desiredAltitude = takeoffResult.data ? takeOffAltitude : desiredAltitude;
            takeoffWatchdog = 0;
        }
    }

    // Publish if fails. Success triggered by State change
    if (!takeoffResult.data)
        pubTakeoffDiagnostic.publish(takeoffResult);
}

void Mavros_commands::cbLand(const std_msgs::EmptyConstPtr){
    std_msgs::Bool landResult;

    landingOrRTL = landResult.data = setMode("LAND");

    // Publish if fails. Success triggered by State change
    if (!landResult.data)
        pubLandDiagnostic.publish(landResult);
}

void Mavros_commands::cbLoiter(const std_msgs::EmptyConstPtr){
    setMode("LOITER");
}

void Mavros_commands::cbDisarm(const std_msgs::EmptyConstPtr){

    requestArm(false);
}

void Mavros_commands::cbGuided(const std_msgs::EmptyConstPtr){
    changeToGuidedMode();
}

void Mavros_commands::cbArm(const std_msgs::EmptyConstPtr){
    requestArm(true);
}

void Mavros_commands::cbUAVPose(const geometry_msgs::PoseStampedConstPtr &msg){
    currentAltitude = msg->pose.position.z;

    if(lastAltitude < currentAltitude + 0.1 && ongoingTakeOff){
        takeoffWatchdog ++;
        if(takeoffWatchdog > 100){
            ROS_ERROR("UAV not climbing");
            ongoingTakeOff = false;
            std_msgs::Bool takeoffResult;
            takeoffResult.data = false;
            pubTakeoffDiagnostic.publish(takeoffResult);
        }
    }
    lastAltitude = currentAltitude;
}

bool Mavros_commands::setMode(std::string mode){
    mavros_msgs::SetMode setMode;
    setMode.request.base_mode   = 0;
    setMode.request.custom_mode = mode;

    if (srvClientSetMode.call(setMode) && setMode.response.mode_sent){
        ROS_INFO_STREAM(mode << " enabled");
        return true;
    }

    ROS_ERROR_STREAM("Failed to set " << mode);
    return false;
}

void Mavros_commands::cbMissionStart(const std_msgs::EmptyConstPtr){
    //    changeToGuidedMode();
    changeToAutoMode();
    onMission = true;
}

void Mavros_commands::cbMissionStop(const std_msgs::EmptyConstPtr){
    if (onMission)
        changeToGuidedMode();
    onMission = false;
}

void Mavros_commands::cbRTL(const std_msgs::EmptyConstPtr){
    landingOrRTL = setMode("RTL");
}

void Mavros_commands::cbState(const mavros_msgs::StateConstPtr &msg){
    /// FROM https://mavlink.io/en/messages/common.html#MAV_STATE
    /// MAV_STATE_STANDBY = 3
    /// MAV_STATE_ACTIVE = 4

    std_msgs::Bool stateChange;
    stateChange.data = true;

    if ((currentState == 3) && (msg->system_status == 4)){
        pubTakeoffDiagnostic.publish(stateChange);
    } else if ((currentState == 4) && (msg->system_status == 3)) {
        if(landingOrRTL){
            pubRequestReached.publish(emptyMsg);
            landingOrRTL = false;
        }
        pubLandDiagnostic.publish(stateChange);
        ongoingTakeOff = false;
    }

    currentState = msg->system_status;
}

void Mavros_commands::cbAuto(const std_msgs::EmptyConstPtr){
    modeAuto = true;
}

void Mavros_commands::cbBrake(const std_msgs::EmptyConstPtr){
    setMode("BRAKE");
}

void Mavros_commands::changeToAutoMode(){
    if (setMode("AUTO"))
        modeAuto = false;
}

void Mavros_commands::changeToGuidedMode(){
    setMode("GUIDED");
}

bool Mavros_commands::requestArm(bool arm){
    mavros_msgs::CommandBool requestArming;
    requestArming.request.value = arm;

    std::string armReqState = arm ? "ARM":"DISARM";

    if (srvClientArming.call(requestArming) && requestArming.response.success){
        ROS_INFO_STREAM("Vehicle " << armReqState);
        return true;
    }

    ROS_ERROR_STREAM(armReqState << " failed");
    return false;
}

bool Mavros_commands::requestTakeoff(){
    mavros_msgs::CommandTOL requestTakeoff;
    requestTakeoff.request.latitude   = 0.0;
    requestTakeoff.request.longitude  = 0.0;
    requestTakeoff.request.altitude   = static_cast<float>(takeOffAltitude);

    std_msgs::Bool takeoffResult;

    if (currentState == 4){
        ROS_INFO_STREAM("Already Flying");
        pubRequestReached.publish(emptyMsg);
        return false;
    }

    if (srvClientTakeOff.call(requestTakeoff) && requestTakeoff.response.success){
        ROS_INFO_STREAM("Takeoff success");
        return true;
    }

    ROS_ERROR_STREAM("Takeoff failed");
    return false;
}
