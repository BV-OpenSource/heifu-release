#include "Planner/Planner.hpp"

Planner::Planner():nh("~"),shm_nh("~")
{
    constructor();
}

Planner::Planner(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):nh(private_nh_),shm_nh(nh_){
    constructor();
}

void Planner::ResetWorkspaceSize(StateSpace *stateSpace){
    geometry_msgs::Point center = stateSpace->getCenter();
    stateSpace->setMaxMin({center.x, center.y, center.z}, workspace_size);
}

void Planner::PrintWorkspaceSize(){
    ROS_INFO("Workspace_size: %f %f %f", workspace_size[0], workspace_size[1], workspace_size[2]);
}

void Planner::constructor()
{
    std::string ns = ros::this_node::getNamespace();

    std::string param_ns = ns+"/Planners_Manager_nodelet/";

    //	Parameters
    nh.param<double>(param_ns+"paramNodeRate", paramNodeRate, 1000.0);
    nh.param<double>(param_ns+"paramGoalTolerance", goalFindTolerance, 1.0);
    nh.param<double>(param_ns+"paramRotationTolerance", rotationTolerance, 0.2);

    nh.param<std::string>(param_ns+"paramMapFrameId", map_frame_id, "map");
    nh.param<std::string>(param_ns+"paramTargetFrame", robot_frame_id, "base_link");
    nh.param<std::string>(param_ns+"paramPubTopicWPReached", reachedTopic, "/waypoint_reached");

    nh.param<std::vector<double> >(param_ns+"paramRobotSize", this->robot_size, std::vector<double>(3,1));
    shm_nh.param<std::vector<double> >(param_ns+"paramWorkspaceSize", this->workspace_size, std::vector<double>(3,50));
    shm_nh.param<std::vector<double> >(param_ns+"paramWorkspaceCenter", this->workspace_center, std::vector<double>(3,25));

    //	Subscribers
    subStop = nh.subscribe(ns + "/planners/stop", 10, &Planner::cbStop, this);
    subPause = nh.subscribe(ns + "/planners/pause", 10, &Planner::cbPause, this);
    subStart = nh.subscribe(ns + "/planners/start", 10, &Planner::cbStart, this);
    subCollisionPause = nh.subscribe(ns + "/planners/collision/pause", 10, &Planner::cbCollisionPause, this);
    subCollisionStart = nh.subscribe(ns + "/planners/collision/start", 10, &Planner::cbCollisionStart, this);
    subSafetyDistance = nh.subscribe(ns + "/planners/updateSafetyDistance", 10, &Planner::cbCollisionStarting, this);

    subNodeletLoop = nh.subscribe(ns + "/nodelet_loop", 1, &Planner::cbNodeletLoop, this);

    subWPReached = nh.subscribe(ns + reachedTopic, 10, &Planner::cbWPReached, this);
    subGoalPosition = nh.subscribe(ns + "/planners/setpoint", 10, &Planner::cbGoalPosition, this);
    subMavrosPose = nh.subscribe(ns + "/mavros/local_position/pose", 10, &Planner::cbMavrosPose, this);

    subMapPtr = shm_nh.subscribe(ns + "/GPU_Voxels/map", 1, &Planner::cbGetMapPtr, this);
    subMapOffset = shm_nh.subscribe(ns + "/GPU_Voxels/offset", 1, &Planner::cbMapOffset, this);
    subMutexPtr = shm_nh.subscribe(ns + "/GPU_Voxels/MutexMap", 1, &Planner::cbGetMutexPtr, this);

    //	Publishers
    pubMarker = nh.advertise<visualization_msgs::Marker>(ns + "/visualization_marker", 10);
    pubSetPoint = nh.advertise <uav_msgs::UAVWaypoint> (ns + "/planners/local_position/setpoint", 10);
    pubRotSetPoint = nh.advertise <geometry_msgs::PoseStamped> (ns + "/mavros/setpoint_position/local", 10);
    pubGoalReached = nh.advertise<std_msgs::Empty>(ns + "/planners/reached_position", 100);

    startLastRead.sec = 0;
}
