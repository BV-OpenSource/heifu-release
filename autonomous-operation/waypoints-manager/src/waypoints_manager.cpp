#include "waypoints_manager/waypoints_manager.hpp"

Waypoints_Manager::Waypoints_Manager():nh("~"){
    ns = ros::this_node::getNamespace();

    // Parameters
    nh.param<double>("paramNodeRate", paramNodeRate, 100);

    nh.param<int>("paramVehicleType",    paramVehicleType,      100);
    nh.param<int>("paramAutopilotType",  paramAutopilotType,    100);

    nh.param<std::string>("paramSubStartPlan",          paramSubStartPlan,          "/start_plan");
    nh.param<std::string>("paramSubReachedPosition",    paramSubReachedPosition,    "/waypoint_reached");
    nh.param<std::string>("paramSubSetpointPoint",      paramSubSetpointPoint,      "/waypointsManager/setpoint");
    nh.param<std::string>("paramSubConvertedGoalPoint", paramSubConvertedGoalPoint, "/gnss_utils/converted_coordinates");

    nh.param<std::string>("paramPubRTL",                paramPubRTL,                "/rtl");
    nh.param<std::string>("paramPubLand",               paramPubLand,               "/land");
    nh.param<std::string>("paramPubToFW",               paramPubToFW,               "/to_FW");
    nh.param<std::string>("paramPubToMC",               paramPubToMC,               "/to_MC");
    nh.param<std::string>("paramPubTakeoff",            paramPubTakeoff,            "/takeoff");
    nh.param<std::string>("paramPubPlannersStart",      paramPubPlannersStart,      "/planners/start");
    nh.param<std::string>("paramPubPoseCommandLLA",     paramPubPoseCommandLLA,     "/gnss_utils/goal_coordinates");
    nh.param<std::string>("paramPubGoalPointOutput",    paramPubGoalPointOutput,    "/waypointsManager/goalWaypoint");
    nh.param<std::string>("paramPubPlannersSetpoint",   paramPubPlannersSetpoint,   "/planners/setpoint");
    nh.param<std::string>("paramPubLandingEnableTopic", paramPubLandingEnableTopic, "/landing/enableLanding");
    nh.param<std::string>("paramPubLandingPadPosition", paramPubLandingPadPosition, "/landingpad/global_position/global");

    // Subscribers
    subStartPlan            = nh.subscribe(ns + paramSubStartPlan,              10, &Waypoints_Manager::cbStartPlan, this);
    subPlanners             = nh.subscribe(ns + "/planners/collision/start",    1,  &Waypoints_Manager::cbPlannersStart, this);
    subPlannersReached      = nh.subscribe(ns + "/planners/reached_position",   1,  &Waypoints_Manager::cbPlannersFinish, this);
    subReachedPosition      = nh.subscribe(ns + paramSubReachedPosition,        10, &Waypoints_Manager::cbReachedPosition, this);
    subSetpointCoordinates  = nh.subscribe(ns + paramSubSetpointPoint,          10, &Waypoints_Manager::cbSetpointCoordinates, this);
    subConvertedCoordinates = nh.subscribe(ns + paramSubConvertedGoalPoint,     10, &Waypoints_Manager::cbConvertedCoordinates, this);

    subHomePosition         = nh.subscribe(ns + "/gnss_utils/home_LLA",             10, &Waypoints_Manager::cbHomePosition, this);
    subGlobalPosition       = nh.subscribe(ns + "/mavros/global_position/global",   10, &Waypoints_Manager::cbGlobalPosition, this);

    // Publishers
    pubRTL              = nh.advertise<std_msgs::Empty>(ns + paramPubRTL, 10);
    pubLand             = nh.advertise<std_msgs::Empty>(ns + paramPubLand, 10);
    pubToFW             = nh.advertise<std_msgs::Empty>(ns + paramPubToFW, 10);
    pubToMC             = nh.advertise<std_msgs::Empty>(ns + paramPubToMC, 10);
    pubTakeoff          = nh.advertise<std_msgs::UInt8>(ns + paramPubTakeoff, 10);
    pubGoalENUPoint     = nh.advertise<uav_msgs::UAVWaypoint>(ns + paramPubGoalPointOutput, 10);
    pubPoseCommandLLA   = nh.advertise<geographic_msgs::GeoPointStamped>(ns + paramPubPoseCommandLLA, 10);

    pubPlannersStart        = nh.advertise<std_msgs::Empty>(ns + paramPubPlannersStart, 10);
    pubEnableLanding        = nh.advertise<std_msgs::Empty>(ns + paramPubLandingEnableTopic, 10);
    pubPlannersSetpoint     = nh.advertise<geometry_msgs::Pose>(ns + paramPubPlannersSetpoint, 10);
    pubLandingPadPosition   = nh.advertise<sensor_msgs::NavSatFix>(paramPubLandingPadPosition, 10);
}

Waypoints_Manager::~Waypoints_Manager(){}

void Waypoints_Manager::Run(){
    takeoffSent = false;
    plannersIsRunning = false;

    ros::Rate go(paramNodeRate);
    while(ros::ok()){
        ros::spinOnce();
        go.sleep();
    }
}

void Waypoints_Manager::cbGlobalPosition(const sensor_msgs::NavSatFix::ConstPtr msg){
    currGlobalPosition = *msg;
}

void Waypoints_Manager::cbHomePosition(const geographic_msgs::GeoPointStamped& msg){
    HomePosition.latitude = msg.position.latitude;
    HomePosition.altitude = msg.position.altitude;
    HomePosition.longitude = msg.position.longitude;
}

void Waypoints_Manager::cbStartPlan(const std_msgs::Empty ){
    currentDesiredPosition = 1;
    vectorWaypoints.waypoints.clear();
    lastPositionIsLand = false;
    takeoffSent = false;
    subWaypointsMavros = nh.subscribe(ns + "/mavros/mission/waypoints",10,&Waypoints_Manager::cbWaypointsList,this);
    ROS_INFO("Waiting for Waypoints list...");
}

void Waypoints_Manager::cbWaypointsList(const mavros_msgs::WaypointListConstPtr& msg){
    vectorWaypoints = *msg;
    vectorWaypointsSize = vectorWaypoints.waypoints.size();
    if(vectorWaypointsSize > 0){
        pubDesiredPosition(vectorWaypoints, currentDesiredPosition);
    }else{
        ROS_WARN("Mission is empty.");
    }
}

void Waypoints_Manager::pubDesiredPosition(mavros_msgs::WaypointList plan, size_t index){
    geometry_msgs::Pose msg;
    std_msgs::UInt8 takeoffAltitude;

    if(index < plan.waypoints.size()){
        waypointRadius = static_cast<float>(plan.waypoints[index].param2);
        loiterRadius = static_cast<float>(plan.waypoints[index].param3);
        waypointCommand = plan.waypoints[index].command;

        ROS_INFO(CYAN"[WP MANAGER]Current waypoint= %d | Command = %d\n Waypoint Radius: %f", index, plan.waypoints[index].command, waypointRadius);

        switch (plan.waypoints[index].command) {
            case TAKEOFF:
            case VTOL_TAKEOFF:
                if(!takeoffSent){
                    takeoffAltitude.data = static_cast<uint8_t>(plan.waypoints[index].z_alt);
                    pubTakeoff.publish(takeoffAltitude);
                    takeoffSent = true;
                }else{
                    pubWaypoint(plan.waypoints[index].frame, currGlobalPosition.latitude, currGlobalPosition.longitude, plan.waypoints[index].z_alt);
                    takeoffSent = false;
                }
                break;
            case LAND:
                if (!lastPositionIsLand){
                    pubWaypoint(plan.waypoints[index].frame, plan.waypoints[index].x_lat, plan.waypoints[index].y_long, plan.waypoints[index].z_alt);
                    if(pubEnableLanding.getNumSubscribers() > 0){
                        landingPad.header.stamp = ros::Time::now();
                        landingPad.latitude = plan.waypoints[index].x_lat;
                        landingPad.longitude = plan.waypoints[index].y_long;
                        landingPad.altitude = HomePosition.altitude;
                        pubLandingPadPosition.publish(landingPad);
                    }
                    lastPositionIsLand=true;
                }
                else{
                    endLanding = true;
                    if(pubEnableLanding.getNumSubscribers() > 0){
                        pubEnableLanding.publish(empty);
                    }else{
                        pubLand.publish(empty);
                    }
                }
                break;
            case RETURNTOLAUNCH:
                pubRTL.publish(empty);
                break;
            case VTOL_TRANSITION:
                if(paramVehicleType == HVTOL){
                    if(static_cast<int>(plan.waypoints[index].param1) == 4)
                        pubToFW.publish(empty);
                    if(static_cast<int>(plan.waypoints[index].param1) == 3)
                        pubToMC.publish(empty);
                }
                break;
            case LOITER:
            case WAYPOINT:
                pubWaypoint(plan.waypoints[index].frame, plan.waypoints[index].x_lat, plan.waypoints[index].y_long, plan.waypoints[index].z_alt);
                break;
            case SPEED_CHANGE:
                missionVelocity = static_cast<float>(plan.waypoints[index].param2);
                this->cbReachedPosition(empty);
                break;
            case TAKEPHOTO:
                ROS_INFO("TAKEPHOTO");
                this->cbReachedPosition(empty);
                break;
            default:
                return;
        }
    }
}

void Waypoints_Manager::pubWaypoint(const mavros_msgs::Waypoint::_frame_type frame, const double x, const double y, const double z){
    geographic_msgs::GeoPointStamped msg;
    msg.header.stamp = ros::Time::now();

    uav_msgs::UAVWaypoint ENUPoint;

    switch (frame) {
        case FRAME_GLOBAL:
            msg.position.latitude = x;
            msg.position.longitude = y;
            msg.position.altitude = z;
            pubPoseCommandLLA.publish(msg);
            break;
        case FRAME_GLOBAL_REL_ALT:
            msg.position.latitude = x;
            msg.position.longitude = y;
            msg.position.altitude = z + HomePosition.altitude;
            pubPoseCommandLLA.publish(msg);
            break;
        case FRAME_LOCAL_ENU:
            ENUPoint.type = uav_msgs::UAVWaypoint::TYPE_VELOCITY_YAW_WP;
            ENUPoint.frame = FRAME_LOCAL_NED;
            ENUPoint.radius = waypointRadius;
            ENUPoint.x = static_cast<float>(x);
            ENUPoint.y = static_cast<float>(y);
            ENUPoint.z = static_cast<float>(z);
            ENUPoint.maxLinearVel = missionVelocity;
            ENUPoint.yawError = 0.27f;
            pubGoalENUPoint.publish(ENUPoint);
            break;
        default:
            ROS_INFO(RED"[WP MANAGER]Waypoint frame not recognized, aborting \n");
            pubRTL.publish(empty);
            return;
    }
}

void Waypoints_Manager::cbSetpointCoordinates(const geographic_msgs::GeoPointConstPtr msg){
    waypointCommand = WAYPOINT;
    waypointRadius = 2.0;
    pubWaypoint(FRAME_GLOBAL_REL_ALT, msg->latitude, msg->longitude, msg->altitude);
}

void Waypoints_Manager::cbConvertedCoordinates(const geometry_msgs::PointStampedConstPtr msg){
    if( pubPlannersSetpoint.getNumSubscribers() > 0 ){
        geometry_msgs::Pose convertedPoint;

        convertedPoint.position.x = msg->point.x;
        convertedPoint.position.y = msg->point.y;
        convertedPoint.position.z = msg->point.z;

        pubPlannersSetpoint.publish(convertedPoint);
        //        pubPlannersStart.publish(empty);
    }
    uav_msgs::UAVWaypoint convertedPoint;

    convertedPoint.maxLinearVel = missionVelocity;
    convertedPoint.x = static_cast<float>(msg->point.x);
    convertedPoint.y = static_cast<float>(msg->point.y);
    convertedPoint.z = static_cast<float>(msg->point.z);
    convertedPoint.frame = FRAME_LOCAL_NED;
    convertedPoint.yawError = 0.27f;

    switch (waypointCommand) {
        case LAND:
        case TAKEOFF:
        case WAYPOINT:
        case VTOL_TAKEOFF:
            convertedPoint.radius = waypointRadius;
            convertedPoint.type = uav_msgs::UAVWaypoint::TYPE_VELOCITY_YAW_WP;
            break;
        case LOITER:
            convertedPoint.radius = loiterRadius;
            convertedPoint.type = uav_msgs::UAVWaypoint::TYPE_LOITER;
            break;
        default:
            ROS_WARN_STREAM("WaypointCommand not recognized " << waypointCommand);
            return;
    }
    pubGoalENUPoint.publish(convertedPoint);
}

void Waypoints_Manager::cbPlannersStart(const std_msgs::Empty){
    ROS_INFO("Planners is running");
    plannersIsRunning = true;
}
void Waypoints_Manager::cbPlannersFinish(const std_msgs::Empty){
    ROS_INFO("Planners stopped");
    plannersIsRunning = false;
}


void Waypoints_Manager::cbReachedPosition(const std_msgs::Empty){
    if (!lastPositionIsLand && !takeoffSent && !plannersIsRunning){
        //            && (currGlobalPosition.altitude + waypointRadius > vectorWaypoints.waypoints[currentDesiredPosition].z_alt + HomePosition.altitude)
        // TODO: check if position is same as waypoint
        currentDesiredPosition++;
    }

    if((currentDesiredPosition < vectorWaypointsSize) && !endLanding) {
        pubDesiredPosition(vectorWaypoints, currentDesiredPosition);
    }
    else{
        ROS_INFO(GREEN"[WP MANAGER]Finished mission\n");
        vectorWaypoints.waypoints.clear();
        lastPositionIsLand = false;
        endLanding=false;
    }
}
