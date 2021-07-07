#include "navigation_controller/navigation_controller.hpp"

using namespace NC;
using namespace uav_msgs;
using namespace mavros_msgs;

Navigation_Controller::Navigation_Controller():nh("~"){
    std::string ns = ros::this_node::getNamespace();

    vc = new VC::Velocity_Controller();
    pc = new PC::Position_Controller();

    // Parameters
    nh.param<double>     ("paramNodeRate",          paramNodeRate,          100.0);
    nh.param<std::string>("paramSubTopicSetpoint",  paramSubTopicSetpoint,  "/local_position");
    nh.param<std::string>("paramPubTopicWPReached", paramPubTopicWPReached, "/waypoint_reached");

    // Subscribers
    subState            = nh.subscribe(ns + "/mavros/state",           1, &Navigation_Controller::cbState,         this);
    subExtendedState    = nh.subscribe(ns + "/mavros/extended_state",  1, &Navigation_Controller::cbExtendedState, this);
    subSetpoint         = nh.subscribe(ns + paramSubTopicSetpoint,     1, &Navigation_Controller::cbSetpoint,      this);

    // Publishers
    pubOffboard         = nh.advertise<std_msgs::Empty>(ns + "/mode_guided", 10);
    pubPosHold          = nh.advertise<std_msgs::Empty>(ns + "/mode_brake",  10);
    pubWaypointReached  = nh.advertise <std_msgs::Empty>(ns + paramPubTopicWPReached,  1);
    pubVelocityBypass   = nh.advertise <mavros_msgs::PositionTarget>(ns + "/mavros/setpoint_raw/local", 10);

    // Services
    srvClientSetParam   = nh.serviceClient <mavros_msgs::ParamSet> (ns + "/mavros/param/set");
    srvClientGetParam   = nh.serviceClient <mavros_msgs::ParamGet> (ns + "/mavros/param/get");
}

Navigation_Controller::~Navigation_Controller(){
    delete vc;
    delete pc;
}

void Navigation_Controller::Run(){

    ros::Rate r(paramNodeRate);

    bool offboardRequest = false;
    uint16_t offboardTimeOut = 0;
    double currentTime;
    while(ros::ok()){
        currentTime = ros::Time::now().toSec();

        if(((currentTime-cbSetpointTime) > RESET_VELOCITY_BYPASS_TIMER) && flagBypass){
            flagBypass = false;
            waypointReachedSkip = true;
            if(currentState.mode != "BRAKE"){
                pubPosHold.publish(msgEmpty);
            }
//            if(currentState.mode != "AUTO.LOITER"){
//                pubPosHold.publish(msgEmpty);
//            }
        }

        if(pc->posSetpointRec || vc->velSetpointRec || flagBypass){
//            if((currentState.mode != "OFFBOARD") && !offboardRequest){
            if((currentState.mode != "GUIDED") && !offboardRequest){
                offboardRequest = true;
                pubOffboard.publish(msgEmpty);
            }else{
                offboardTimeOut++;
                if(offboardTimeOut == 150){
                    offboardRequest = false;
                    offboardTimeOut = 0;
                }
            }
        }
        if( ( !pc->posSetpointRec && !vc->velSetpointRec && !flagBypass) && flagWaypointReceived && !waypointReachedSkip){
            ROS_INFO("Waypoint finished!");
            pubWaypointReached.publish(msgEmpty);
            pubPosHold.publish(msgEmpty);

            flagWaypointReceived = false;
            offboardRequest = false;
        }
        ros::spinOnce();
        r.sleep();
    }
}

void Navigation_Controller::cbState(const mavros_msgs::State::ConstPtr msg){
    currentState = *msg;
}

void Navigation_Controller::cbExtendedState(const mavros_msgs::ExtendedState::ConstPtr msg){
    currentExtendedState = *msg;
}

void Navigation_Controller::cbSetpoint(const uav_msgs::UAVWaypoint &msg){

    cbSetpointTime= ros::Time::now().toSec();

    switch(msg.type){
        case UAVWaypoint::TYPE_POSITION:
        case UAVWaypoint::TYPE_POSITION_ONLY:
        case UAVWaypoint::TYPE_POSITION_YAW:
        case UAVWaypoint::TYPE_POSITION_YAW_WP:
        case UAVWaypoint::TYPE_YAW_ONLY:
        case UAVWaypoint::TYPE_POSITION_YAW_FIRST_WP:
            ROS_INFO("Position controller activated");
            flagWaypointReceived = true;      // Activate waypoint flag
            waypointReachedSkip  = false;

            pc->flagWaypointReceived = true;  // Deactivate dummy message to position control
            pc->posSetpointRec = true;        // Make sure position control is ON

            vc->velSetpointRec = false;       // Make sure velocity control is OFF

            flagBypass = false;

            if(msg.maxLinearVel >= FLT_EPSILON){
                change_parameter("WPNAV_SPEED", static_cast<double>(msg.maxLinearVel)*100);
            }

            pc->angleInc = 0;
            pc->distanceInc = 0;

            pc->stage = 0;
            pc->goalPoint = msg;
            break;

        case UAVWaypoint::TYPE_VELOCITY:
        case UAVWaypoint::TYPE_VELOCITY_ONLY:
        case UAVWaypoint::TYPE_VELOCITY_YAW:
        case UAVWaypoint::TYPE_VELOCITY_YAW_WP:
        case UAVWaypoint::TYPE_VELOCITY_YAW_FIRST_WP:
            ROS_INFO("Velocity controller activated");
            flagWaypointReceived = true;      // Activate waypoint flag
            waypointReachedSkip  = false;

            pc->flagWaypointReceived = true;  // Deactivate dummy message to position control
            pc->posSetpointRec = false;       // Make sure position control is OFF

            vc->velSetpointRec = true;        // Make sure velocity control is ON

            flagBypass = false;

            vc->angleInc = 0;
            vc->distanceInc = 0;

            vc->stage = 0;                    //Reset the stage when new setpoint is given
            vc->goalPoint = msg;
            break;

        case UAVWaypoint::TYPE_VELOCITY_BYPASS:
            ROS_INFO("Velocity bypass activated");
            pc->flagWaypointReceived = false;  // Deactivate dummy message to position control
            pc->posSetpointRec = false;        // Make sure position control is OFF

            vc->velSetpointRec = false;        // Make sure velocity control is OFF

            flagBypass = true;

            velBypass.coordinate_frame = msg.frame;
            velBypass.type_mask = (~(PositionTarget::IGNORE_VX |
                                     PositionTarget::IGNORE_VY |
                                     PositionTarget::IGNORE_VZ |
                                     PositionTarget::IGNORE_YAW_RATE)) & 0x0FFF;
            velBypass.velocity.x = static_cast<double>(msg.x);
            velBypass.velocity.y = static_cast<double>(msg.y);
            velBypass.velocity.z = static_cast<double>(msg.z);

            pubVelocityBypass.publish(velBypass);
            break;

//        case UAVWaypoint::TYPE_CHANGE_PARAMS:
//            flagWaypointReceived = true;      // Activate waypoint flag
//            waypointReachedSkip  = false;
//            change_parameter("WPNAV_SPEED", msg.radius*100);
//            vc->changeMaxSpeed(msg.radius);
//            break;
        default :
            ROS_WARN("Waypoint type not recognized!");
            return;
    }
}

void Navigation_Controller::change_parameter(std::string string_rec, double value){

    mavros_msgs::ParamSet param_send;

    param_send.request.param_id   = string_rec;
    param_send.request.value.real = value;

    if(!srvClientSetParam.call(param_send)) {
        ROS_ERROR("Failed to set parameter");
    }
}

double Navigation_Controller::get_parameter(std::string string_rec){

    mavros_msgs::ParamGet param_get;

    param_get.request.param_id = string_rec;
    srvClientGetParam.call(param_get);

    return param_get.response.value.real;
}
