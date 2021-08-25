#include "RRT/RRT.hpp"

using namespace RRT3D;

RRT::RRT():n("~") {
    RRT(n, n);
}

RRT::RRT(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):n(private_nh_),shm_nh(nh_){
    std::string ns = ros::this_node::getNamespace();

    //	Parameters
    n.param<bool>  ("paramBiasedRRT",     paramBiasedRRT, true);

    n.param<double>("paramRRTStep",         paramRRTStep,       1.0);
    n.param<double>("paramBiasedRRTProb",   paramBiasedRRTProb, 0.5);
}

void RRT::Setup(){
    PlannerState = Standby;

    // for generating a random number
    srand (static_cast <unsigned> (time(nullptr)));
    stateSpace = new StateSpace(workspace_center, workspace_size);

    // RRT Tree pointer
    rrt = nullptr;

    // Final and Intermediate goals
    goalNode     = nullptr;
    TempGoalNode = nullptr;

    // Find Clear Path and reach Final Goal
    TempGoalReached = true;
    goalFound       = false;
    TempGoalSet     = false;
    pathFound       = false;

    rotationSent = false;

    goalSetpoint.x = FLT_MAX;
    goalSetpoint.y = FLT_MAX;
    goalSetpoint.z = FLT_MAX;

    goalSetpoint.maxAngularVel = 0.785f;

    goalSetpoint.radius = static_cast<float>(goalFindTolerance);
    // Set heading to WP, then goto Position
    goalSetpoint.type = 14;
    //FRAME_LOCAL_NED
    goalSetpoint.frame = 1;

#ifdef VIEW_RVIZ
    goal.type = visualization_msgs::Marker::SPHERE;

    goal.header.frame_id = map_frame_id;
    goal.header.stamp = ros::Time::now();

    goal.ns = "markers";
    goal.id = 3;

    goal.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker
    goal.scale.x = goal.scale.y = 0.5;
    goal.scale.z = 0.1;

    goal.color.r = 1.0f;
    goal.color.g = 0.0f;
    goal.color.b = 0.0f;
    goal.color.a = 1.0;

    goal.lifetime = ros::Duration();

    workSpace.type = visualization_msgs::Marker::CUBE;
    vertices.type  = visualization_msgs::Marker::POINTS;
    sp_edges.type  = visualization_msgs::Marker::POINTS;
    edges.type     = visualization_msgs::Marker::LINE_LIST;

    workSpace.header.frame_id = vertices.header.frame_id = sp_edges.header.frame_id = edges.header.frame_id = map_frame_id;
    workSpace.header.stamp = vertices.header.stamp = sp_edges.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = edges.ns = "vertices_and_lines";
    sp_edges.ns = "pathLines";
    workSpace.action = vertices.action = sp_edges.action = edges.action = visualization_msgs::Marker::ADD;
    workSpace.pose.orientation.w = vertices.pose.orientation.w = sp_edges.pose.orientation.w = edges.pose.orientation.w = 1.0;

    workSpace.ns = "WorkSpace";
    workSpace.id = 0;
    workSpace.color.b = 1.0f;
    workSpace.color.a = 0.3f;
    workSpace.scale.x = workspace_size[0];
    workSpace.scale.y = workspace_size[1];
    workSpace.scale.z = workspace_size[2];
    workSpace.pose.position = stateSpace->getCenter();

    vertices.id = 0;
    edges.id    = 1;
    sp_edges.id = 2;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.05;
    vertices.scale.y = 0.05;
    sp_edges.scale.x = 0.1;
    sp_edges.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.02; //tune it yourself

    // Points are green
    vertices.color.g = 1.0f;
    vertices.color.a = 1.0;

    // Line list is red
    sp_edges.color.g = sp_edges.color.b = edges.color.r = 1.0;
    sp_edges.color.a = edges.color.a = 1.0;
#endif

    nodesInserted = 0;

    ROS_INFO("RRT Node Ready");
}

void RRT::GetGoalWithinSS(){

    if(goalNode != nullptr){
        TempGoalNode = stateSpace->getFreeGoalPoint(goalNode, paramRRTStep/2);

        if(PlannerState == Planning && TempGoalNode != nullptr){
            TempGoalSet = true;
        }
    }else{
        ROS_ERROR("GoalPoint not defined.");
    }
}

void RRT::find_goal()
{
    // Find the goal using RTT
    if (!goalFound) {

        // If there is waypoints to treat
//        ROS_INFO_THROTTLE(3,"PATH %d %d %d", pathFound, smoothenedRobotPath.size()>0, TempGoalReached);
//        if (TempGoalNode != nullptr)
//            ROS_INFO_THROTTLE(3,"TEMP %f %f %f", TempGoalNode->x, TempGoalNode->y, TempGoalNode->z);
        if(pathFound && smoothenedRobotPath.size()>0 && TempGoalReached){
            smoothenedRobotPath[0]->printNode("GO TO");
            coordBackup = smoothenedRobotPath[0]->getNodeAsPoint();

            goalSetpoint.x = static_cast<float>(coordBackup.x);
            goalSetpoint.y = static_cast<float>(coordBackup.y);
            goalSetpoint.z = static_cast<float>(coordBackup.z);

            pubSetPoint.publish(goalSetpoint);

            TempGoalReached = false;
        }

        // Visualization
        if (TempGoalNode != nullptr)
            goal.pose.position = TempGoalNode->getNodeAsPoint();

        double NearestDistance = std::numeric_limits<double>::max();
        Node *closestNode= nullptr;

        if (!pathFound)
        {
            // Adding goalNode bias
            double randomNumber = (static_cast<double>(rand()) / (RAND_MAX));

            //      ROS_WARN("Random number %f", randomNumber);

            Node *randomNode = nullptr;
            if (paramBiasedRRT && (randomNumber < paramBiasedRRTProb) && (TempGoalNode != nullptr)) {
                randomNode = TempGoalNode;
//                ROS_INFO("USING GOAL %f %f %f", TempGoalNode->x, TempGoalNode->y, TempGoalNode->z);
            } else {
                randomNode = stateSpace->genRandomNodeInSpace();
            }

            closestNode = rrt->getNearestNeighbor(randomNode, stateSpace, NearestDistance);
            if(closestNode == nullptr){
                geometry_msgs::Point temp = stateSpace->getCenter();
                closestNode = new Node(temp);
                rrt->GetRoot()->getDeleted();
                rrt = new RRT_Tree();
                rrt->insert(closestNode, nullptr);
            }

//            if ((closestNode != nullptr) && (randomNode != nullptr)){
//                closestNode->printNode("Closest");
//                ROS_INFO("BOTH VALID");
//            }else
//                ROS_INFO("ONE INVALID");

            if(AddNode(randomNode, closestNode)){
                nodesInserted++;
            }
        }

        // If the goal is close enough
        NearestDistance = std::numeric_limits<double>::max();
        closestNode = rrt->getNearestNeighbor(TempGoalNode, stateSpace, NearestDistance);

        //        ROS_INFO("VALID TO GOAL ENTER %f %f %d", NearestDistance, goalFindTolerance, nodesInserted);
        //        if ((closestNode != nullptr) && (NearestDistance > goalFindTolerance))
        //            ROS_INFO("CLOSEST %f %f %f", closestNode->x, closestNode->y, closestNode->z);

        if((closestNode != nullptr) && (NearestDistance <= goalFindTolerance) && TempGoalReached) {
            //calculate shortest path
            Node *tempRobot = new Node(varRobot.pose.position);

            //      if(!tempRobot->closeTo(rrt->GetRoot(), 1))
            //      {
            //        tempRobot->children.push_back(rrt->GetRoot());
            //        rrt->GetRoot()->parent = tempRobot;
            //        rrt->SetRoot(tempRobot);
            //      }

            // WHY NOT substitute only by 'roughShortestPath = rrt->extractFullPath(TempGoalNode, tempRobot, stateSpace);'
            if(!closestNode->closeTo(TempGoalNode, 0.01)){
                rrt->insert(TempGoalNode, closestNode);
                roughShortestPath = rrt->extractFullPath(TempGoalNode, tempRobot, stateSpace);
            }else{
                roughShortestPath = rrt->extractFullPath(closestNode,  tempRobot, stateSpace);
            }

            if(roughShortestPath.size() > 0){
                nodesInserted = 0;
                std::reverse(roughShortestPath.begin(), roughShortestPath.end());

                smoothenedRobotPath = stateSpace->smoothenPath(roughShortestPath);

                pathFound = true;
                //                TempGoalReached = false;
#ifdef VIEW_RVIZ
                edges.points.push_back(closestNode->getNodeAsPoint());
                edges.points.push_back(TempGoalNode->getNodeAsPoint());
#endif
            }else{
                smoothenedRobotPath.clear();// = roughShortestPath;
                //				rrt->GetRoot()->getDeleted();
                rrt->insert(tempRobot, nullptr);
                IgnoreSafety = true;
            }

            //      delete tempRobot;
        }
        else {
            if(nodesInserted > 1000)
            {
                if(goalNode != nullptr){
                    GetGoalWithinSS();

                    //          pathFound = false;

                    ROS_ERROR("REACHED 1000 iterations");

                    if (TempGoalNode == nullptr)
                    {
                        TempGoalSet = false;
                        ROS_ERROR("TempGoalNode nullptr");
                        PlannerState = Standby;
                    }
                }
                nodesInserted = 0;
            }
        }
    }else{
        ROS_INFO("Debug find!");
    }
}

bool RRT::AddNode(Node *randomNode, Node *closestNode){
    Node *newNode = rrt->extendNode(closestNode, randomNode, paramRRTStep);

    if (!stateSpace->isObstructed(newNode) && !stateSpace->edgeIsObstructed(closestNode, newNode)) {
        IgnoreSafety = false;
        rrt->insert(newNode, closestNode);
#ifdef VIEW_RVIZ
        vertices.points.push_back(newNode->getNodeAsPoint());    //for drawing vertices
        edges.points.push_back(closestNode->getNodeAsPoint());    //for drawing edges. The line list needs two points for each line
        edges.points.push_back(newNode->getNodeAsPoint());
#endif
        return true;
    }
    if(IgnoreSafety && !stateSpace->isObstructed(newNode, IgnoreSafety) && !stateSpace->edgeIsObstructed(closestNode, newNode, IgnoreSafety)) {
        rrt->insert(newNode, closestNode);
#ifdef VIEW_RVIZ
        vertices.points.push_back(newNode->getNodeAsPoint());    //for drawing vertices
        edges.points.push_back(closestNode->getNodeAsPoint());    //for drawing edges. The line list needs two points for each line
        edges.points.push_back(newNode->getNodeAsPoint());
#endif
        return true;
    }
    return false;
}

void RRT::cbGoalPosition(const geometry_msgs::Pose &msg){

    if(goalNode != nullptr){ // Ensure goalNode is allocated propperly
        goalNode->setAs(msg.position);
    }else{
        goalNode = new Node(msg.position);
    }

#ifdef VIEW_RVIZ
    goal.pose.position    = msg.position;
    goal.pose.orientation = msg.orientation;
#endif

    // Reset variables
    goalFound = false;

    smoothenedRobotPath.clear();
    roughShortestPath.clear();
    varSetPoint.pose.position.x = 0;
    varSetPoint.pose.position.y = 0;
    varSetPoint.pose.position.z = 0;

    pathFound       = false;
    //    TempGoalReached = false;

    ROS_INFO("Goal Position Msgs: %f %f %f", msg.position.x, msg.position.y, msg.position.z);
    ROS_INFO("Goal Position Node: %f %f %f", goalNode->x, goalNode->y, goalNode->z);

    ros::Time now = ros::Time::now();
    ros::Duration sinceLast = now - startLastRead;
    if( sinceLast.sec < 2){
        cbStart(emptyMsg);
    }
}

void RRT::cbWPReached(const std_msgs::Empty){
    Node* Pose = new Node(varRobot.pose.position);
    if(smoothenedRobotPath.size()>0){
        Pose->printNode("Pose");
        smoothenedRobotPath.at(0)->printNode("Goal");
        ROS_INFO_STREAM(" Reached? " << TempGoalReached);
        if(Pose->closeTo(smoothenedRobotPath.at(0), goalFindTolerance)){
            smoothenedRobotPath.erase(smoothenedRobotPath.begin());
            TempGoalReached = true;

            if(Pose->closeTo(goalNode, goalFindTolerance/2)){
                goalFound = true;
            }
            if( smoothenedRobotPath.size()>0 ){
                //                smoothenedRobotPath.at(0)->printNode("New Goal");
                return;
            }
        }
    }
    TempGoalReached	= true;
    if(goalNode != nullptr && pathFound){
        GetGoalWithinSS();

        if (TempGoalNode == nullptr)
        {
            TempGoalSet = false;
            ROS_ERROR("TempGoalNode nullptr");
            PlannerState = Standby;
        }
    }
    pathFound = false;
    delete Pose;
}

void RRT::cbMavrosPose(const geometry_msgs::PoseStamped &msg){
    if(stateSpace != nullptr){
        stateSpace->poseUAVNode->setAs(msg.pose.position);

        if(rrt == nullptr){
            rrt = new RRT_Tree();
            Node *Start = new Node(stateSpace->poseUAVNode->getNodeAsPoint());
            rrt->insert(Start, nullptr);
        }

        varRobot.pose = msg.pose;

        Node *temp = new Node(stateSpace->getCenter());

        if(!stateSpace->poseUAVNode->closeTo(temp, 1)){
            auto q = varRobot.pose.orientation;
            double robotTheta = atan2(2*(q.x*q.y + q.w*q.z), 1-2*(q.y*q.y + q.z*q.z));
            stateSpace->setMaxMin({msg.pose.position.x, msg.pose.position.y, msg.pose.position.z}, workspace_size, workspace_center, robotTheta);
#ifdef VIEW_RVIZ
            workSpace.pose.position = stateSpace->getCenter();
            workSpace.pose.orientation.z = q.z;
            workSpace.pose.orientation.w = q.w;
#endif

            //    Node *PrevTemp = new Node(0,0,0);

            //    if (TempGoalNode != nullptr)
            //      PrevTemp = TempGoalNode;

            //    delete PrevTemp;
        }
        delete temp;
#ifdef VIEW_RVIZ
        sp_edges.points.push_back(msg.pose.position);
#endif
    }
}

void RRT::cbStop(const std_msgs::Empty ){
    PlannerState = Stop;
}

void RRT::cbPause(const std_msgs::Empty ){
    pathFound=false;
    PlannerState = Pause;
    TempGoalSet = false;
    TempGoalReached = true;
    rotationSent = false;
    smoothenedRobotPath.clear();
    //roughShortestPath.clear();
    varSetPoint.pose.position.x = 0;
    varSetPoint.pose.position.y = 0;
    varSetPoint.pose.position.z = 0;
    varSetPoint.pose.orientation.x = 0;
    varSetPoint.pose.orientation.y = 0;
    varSetPoint.pose.orientation.z = 0;
    varSetPoint.pose.orientation.w = 1;
    ROS_INFO("RRT Paused");
#ifdef VIEW_RVIZ
    //  if(rrt != nullptr){
    //    edges.points.clear();
    //  }
#endif
}

void RRT::cbCollisionPause(const std_msgs::Empty msg){
//    if(PlannerState == Planning){
        cbPause(msg);
        PlannerState = Colliding;
//    }
}

void RRT::cbCollisionStart(const std_msgs::Empty msg){
//    if(PlannerState == Colliding){
        cbStart(msg);
//    }
}

void RRT::cbGetMapPtr(const std_msgs::UInt64 &msg)
{
    if(stateSpace != nullptr){
        ObstaclePtr = new Obstacle(msg.data);
        stateSpace->addObstacle(ObstaclePtr);
        std_msgs::Empty msg;
        cbCollisionStarting(msg);
        subMapPtr.shutdown();
    }
}

void RRT::cbGetMutexPtr(const std_msgs::UInt64 &msg)
{
    if (ObstaclePtr != nullptr)
    {
        ObstaclePtr->mapAccessMutex = reinterpret_cast<std::mutex*>(msg.data);
        subMutexPtr.shutdown();
    }
}

void RRT::cbMapOffset(const geometry_msgs::PointStampedPtr msg){
    mapCenter.x = static_cast<float>(msg->point.x);
    mapCenter.y = static_cast<float>(msg->point.y);
    mapCenter.z = static_cast<float>(msg->point.z);
    if(stateSpace != nullptr){
        stateSpace->updateObstacleMapCenter(mapCenter);
    }
}

void RRT::cbCollisionStarting(const std_msgs::Empty){
    double safetyDistance;
    std::string ns = ros::this_node::getNamespace();
    if(ros::param::get(ns+"/collision_avoidance_nodelet/paramSafetyDistance", safetyDistance)){
        if(stateSpace != nullptr){
            stateSpace->updateObstacleSafetyDistance(safetyDistance);
        }
    }else{
        ROS_WARN("SafetyDistance param could not be readed.");
    }
}

void RRT::cbStart(const std_msgs::Empty ){
    startLastRead = ros::Time::now();

    IgnoreSafety = true;
    PlannerState = Planning;
    GetGoalWithinSS();

    // If no valid goalNode exists
    if (goalNode == nullptr)
    {
        ROS_ERROR("goalNode nullptr");
        PlannerState = Standby;
    }
    //  else
    //  {
    //    goalFound=false;
    //    pathFound=false;
    //  }
}

void RRT::cbNodeletLoop(const std_msgs::Empty){
    // Both ROS Shutdown and Planner Stop end the operation
    if(ros::ok() && PlannerState != Stop){
        if(!goalFound){
            ROS_INFO_ONCE("Starting...");

            if(TempGoalSet){
                find_goal();
            }
        }else{
            if(rrt != nullptr){
                std_msgs::Empty msg;
                pubGoalReached.publish(msg);
                ROS_INFO("Goal Reached!");
                goalNode->getDeleted();
                goalNode = nullptr;
                rrt->GetRoot()->getDeleted();
                delete rrt;
                rrt = nullptr;
                goalFound = false;
                TempGoalSet = false;
                PlannerState = Standby;
                edges.points.clear();
                vertices.points.clear();

            }
        }
#ifdef VIEW_RVIZ
        // publish these messages to ROS system
        pubMarker.publish(goal);

        //publish msgs
        pubMarker.publish(vertices);
        pubMarker.publish(edges);
        pubMarker.publish(workSpace);
        pubMarker.publish(sp_edges);
#endif
    }else{
        std::cout<< "RRT terminating..." << std::endl;
        subMavrosPose.shutdown();
        if(rrt != nullptr){
            rrt->GetRoot()->getDeleted();
            delete rrt;
            rrt = nullptr;
        }
        stateSpace->cleanObstacles();
        delete stateSpace;
        stateSpace = nullptr;
        subNodeletLoop.shutdown();
        std::cout<< "RRT terminated!" << std::endl;
    }
}

// Not used in nodelets
void RRT::Run(){

    ros::Rate go(paramNodeRate);

    //	bool gotMap = false;
    while (ros::ok() && PlannerState != Stop){
        if(!goalFound){
            ROS_INFO_ONCE("Starting...");

            if(TempGoalSet){
                find_goal();
            }
        }else{
            if(rrt != nullptr){
                std_msgs::Empty msg;
                pubGoalReached.publish(msg);
                ROS_INFO("Goal Reached!");
                goalNode->getDeleted();
                goalNode = nullptr;
                rrt->GetRoot()->getDeleted();
                delete rrt;
                rrt = nullptr;
                goalFound = false;
                TempGoalSet = false;
                PlannerState = Standby;
            }
        }
        // publish these messages to ROS system
        pubMarker.publish(goal);

        //publish msgs
        pubMarker.publish(vertices);
        pubMarker.publish(edges);
        pubMarker.publish(workSpace);

        //ros spins, force ROS frame to refresh/update once
        ros::spinOnce();

        go.sleep();
    }
    std::cout<< "RRT Run terminating..." << std::endl;
    subMavrosPose.shutdown();
    if(rrt != nullptr){
        rrt->GetRoot()->getDeleted();
        delete rrt;
        rrt = nullptr;
    }
    stateSpace->cleanObstacles();
    delete stateSpace;
    stateSpace = nullptr;
}
