#include "Planner/StateSpace.hpp"

StateSpace::StateSpace(std::vector<double> center, std::vector<double> size, double angle)
{
    setMaxMin(center,size);
    rotation = angle;

    poseUAVNode = new Node(0,0,0);
}

//todo: check if the node exists in the tree already
Node * StateSpace::genRandomNodeInSpace(){
    Node* temp = new Node(StateSpace::generateRandomNumber(min_x, max_x),
                          StateSpace::generateRandomNumber(min_y, max_y),
                          StateSpace::generateRandomNumber(min_z, max_z));

    geometry_msgs::Point w;
    w = getRotatedVectorToCenter(temp, -rotation);
    delete temp;
    w.x += this->_center.x;
    w.y += this->_center.y;
    w.z += this->_center.z;
    temp = new Node(w);
    return temp;
}

double StateSpace::generateRandomNumber(double min, double max) {
    return  (min + (rand() / (RAND_MAX/(max - min))));
}

bool StateSpace::isGoalWithinStateSpace(Node *node, const double margin) {
    geometry_msgs::Point w;

    w = getRotatedVectorToCenter(node, rotation);

    w.x += this->_center.x;
    w.y += this->_center.y;
    w.z += this->_center.z;

    return ((w.x >= min_x + margin && w.x <= max_x - margin) &&
            (w.y >= min_y + margin && w.y <= max_y - margin) &&
            (w.z >= min_z + margin && w.z <= max_z - margin));
}

bool StateSpace::isWithinStateSpace(Node *node) {
    geometry_msgs::Point w = getRotatedVectorToCenter(node, rotation);
    w.x += this->_center.x;
    w.y += this->_center.y;
    w.z += this->_center.z;

    if(((w.x >= min_x && w.x <= max_x) && (w.y >= min_y && w.y <= max_y) && (w.z >= min_z && w.z <= max_z))){
        return true;
    }else{
        return false;
    }
}

bool StateSpace::isGoalObstructed(Node *node, const double margin) {
    if (!isGoalWithinStateSpace(node, margin)){
        return true;
    }else {
        for (std::list<Obstacle *>::iterator it = obstacles.begin(); it != obstacles.end(); it++) {
            if ((*it)->isWithinObstacle(node)) {
                return true;
            }
        }
    }
    return false;
}

bool StateSpace::isEdgeGoalObstructed(Node *init, Node *goal, double margin, bool ignoreSafety) {
    if (!isGoalWithinStateSpace(goal, margin)){
        return true;
    }

    return edgeIsObstructed(init, goal, ignoreSafety);
}

bool StateSpace::isObstructed(Node *node, bool ignoreSafety) {

    if (!isWithinStateSpace(node)){
        return true;
    }else {
        for (std::list<Obstacle *>::iterator it = obstacles.begin(); it != obstacles.end(); it++) {
            if ((*it)->isWithinObstacle(node, ignoreSafety)) {
                //				node->getDeleted();
                return true;
            }
        }
    }
    return false;
}

void StateSpace::addObstacle(Obstacle *obstacle) {
    obstacles.push_back(obstacle);
}

void StateSpace::cleanObstacles()
{
    while(this->obstacles.size()){
        Obstacle *temp = this->obstacles.back();
        this->obstacles.pop_back();
        delete temp;
    }
    obstacles.clear();
}

void StateSpace::updateObstacleMapCenter(Vector3f mapCenter){
    if(this->obstacles.size()>0){
        Obstacle *temp = this->obstacles.back();
        temp->updateMapCenter(mapCenter);
    }
}

void StateSpace::updateObstacleSafetyDistance(double newDistance){
    if(this->obstacles.size()>0){
        Obstacle *temp = this->obstacles.back();
        temp->updateSafetyDistance(newDistance);
    }
}

bool StateSpace::edgeIsObstructed(Node *nearestNode, Node *newNode, bool ignoreSafety) {
    for (std::list<Obstacle *>::iterator it = obstacles.begin(); it != obstacles.end(); it++) {
        if ((*it)->edgeWithinObstacle(nearestNode, newNode, ignoreSafety)) {
            return true;
        }
    }
    return false;
}

bool StateSpace::nodeInSpace(Node *leaf){
    if (leaf == nullptr) {
        return false;
    } else {
        for (std::list<Node *>::iterator it = leaf->children.begin(); it != leaf->children.end(); it++){
            if(!this->isWithinStateSpace(*it)){
                if(nodeInSpace(*it)){
                    return true;
                }
            }else{
                return true;
            }
        }
        return false;
    }
}

std::vector<Node *> StateSpace::smoothenPath(std::vector<Node *> roughShortestPath) {
    std::vector<Node *> robotPath;
    // assumes the goal is at the back
    // going from goal up
    Node *firstNode = roughShortestPath.back();
    roughShortestPath.pop_back();
    Node *prev = roughShortestPath.back();
    roughShortestPath.pop_back();
    Node *next = roughShortestPath.back();
    robotPath.push_back(firstNode);

    while (roughShortestPath.size() > 0) {
        if (edgeIsObstructed(firstNode, next, false) || !isWithinStateSpace(next)) {
            robotPath.push_back(prev);
            firstNode = prev;
        }
        prev = next;
        roughShortestPath.pop_back();
        next = roughShortestPath.back();
    }
    // adding the start
    std::reverse(robotPath.begin(), robotPath.end());
    std::cout<< "printing the NodePath Final\n";
    for (size_t i = 0; i < robotPath.size(); ++i) {
        robotPath.at(i)->printNode("Final");
    }
    return robotPath;
}

void StateSpace::setMaxMin(std::vector<double> workspace_center, std::vector<double> workspace_size, std::vector<double> centerOffset, double angle){
    if(workspace_center.size() > 0 && workspace_size.size() > 0){
        this->min_x = workspace_center[0]-workspace_size[0]/2+centerOffset[0];
        this->min_y = workspace_center[1]-workspace_size[1]/2+centerOffset[1];
        this->min_z = workspace_center[2]-workspace_size[2]/2+centerOffset[2];

        this->max_x = workspace_center[0]+workspace_size[0]/2+centerOffset[0];
        this->max_y = workspace_center[1]+workspace_size[1]/2+centerOffset[1];
        this->max_z = workspace_center[2]+workspace_size[2]/2+centerOffset[2];

        this->_center.x = workspace_center[0];
        this->_center.y = workspace_center[1];
        this->_center.z = workspace_center[2];

        this->_size.x = workspace_size[0];
        this->_size.y = workspace_size[1];
        this->_size.z = workspace_size[2];
    }else{
        ROS_ERROR("Workspace Center or Size is empty!");
    }
    this->rotation = angle;
}

Node *StateSpace::getClosestPoint(Node *goalNode, const double margin, uint8_t &planeAxis){
    if(!(this->isGoalObstructed(goalNode, margin))){
        return goalNode;
    }else{
        geometry_msgs::Point v = getRotatedVectorToCenter(goalNode, 0);
        geometry_msgs::Point w = getRotatedVectorToCenter(goalNode, rotation);
        geometry_msgs::Point geometricCenter = this->getGeometricCenter();
        Node *center = new Node(geometricCenter);
        geometry_msgs::Point offset = getRotatedVectorToCenter(center, 0);

        double scale = (this->_size.x/2 + offset.x*w.x/abs(w.x))/abs(w.x);
        scale *= 0.8;
        Node *temp = new Node(this->_center.x + v.x*scale, this->_center.y + v.y*scale, this->_center.z + v.z*scale);
        if(this->isGoalWithinStateSpace(temp, margin)){
            planeAxis = 2;
            delete center;
            return temp;
        }

        scale = (this->_size.y/2 + offset.y*w.y/abs(w.y))/abs(w.y);
        scale *= 0.8;
        temp->setAs(this->_center.x + v.x*scale, this->_center.y + v.y*scale, this->_center.z + v.z*scale);
        if(this->isGoalWithinStateSpace(temp, margin)){
            planeAxis = 1;
            delete center;
            return temp;
        }

        scale = (this->_size.z/2 + offset.z*w.z/abs(w.z))/abs(w.z);
        scale *= 0.8;
        temp->setAs(this->_center.x + v.x*scale, this->_center.y + v.y*scale, this->_center.z + v.z*scale);
        if(this->isGoalWithinStateSpace(temp, margin)){
            planeAxis = 3;
            delete center;
            return temp;
        }
        delete center;
        delete temp;
        ROS_ERROR("END");
        return nullptr;
    }
}

Node* StateSpace::getFreeGoalPoint(Node *goalNode, const double margin){

    Node *TempGoalNode = nullptr;
    if(goalNode != nullptr){
        uint8_t planeFace = 0;
        geometry_msgs::Point coordBackup;
        if(!this->isGoalWithinStateSpace(goalNode, margin)){
            // Missing if goalNode is obstructed
            TempGoalNode = this->getClosestPoint(goalNode, margin, planeFace);
        }else{
            TempGoalNode = new Node(0,0,0);
            TempGoalNode->copy(goalNode);
            return TempGoalNode;
        }
        if(TempGoalNode == nullptr){
            ROS_ERROR("GoalPoint could not be defined!");
            return TempGoalNode;
        }else{
            coordBackup = TempGoalNode->getNodeAsPoint();
        }

        double x = 0.0;
        double y = 0.0;
        double angle = 0.1;

        // Space between the spirals
        double b = 0.5;
        double arcLength = 0.75;
        tf::Vector3 rotation(0,0,0);
        if(planeFace>0){
            rotation[planeFace-1] = 1;
        }
        rotation *= -M_PI_2;
        double radius = b * angle;
        double lastAngle = angle;

        uint8_t tries = 0;
        bool    signedPlane = true;
        uint8_t firstPlane  = planeFace;

        while(this->isEdgeGoalObstructed(poseUAVNode, TempGoalNode, margin, true)){
            angle = sqrt(radius*radius + 2*b*arcLength)/b;

            // Ignore angles between 190 and 350 degrees
            //ROS_WARN("ANGLE1 %f", std::fmod(angle*180/M_PI,360));
            //			double temp_angle = std::fmod(angle,(2*M_PI));
            //			if(temp_angle>3.32 && temp_angle<6.11){
            //				int nLaps = int(angle/M_PI)-1;
            //				angle = 6.11 + M_PI*nLaps;
            //			}
            //ROS_WARN("ANGLE2 %f", std::fmod(angle*180/M_PI,360));

            radius = b * angle;
            x = radius * sin(angle);
            y = -radius * cos(angle);

            //			ROS_WARN_STREAM("XY1 "<<x<<" "<<y<<" "<<radius);
            double temp_angle = std::fmod(angle, M_PI_2);
            if(x < -0.1 && y > 0.0){
                //x *= -1;
                angle += M_PI-temp_angle*2;
                radius = b * angle;
                x = radius * sin(angle);
                y = -radius * cos(angle);
            }
            //			ROS_WARN_STREAM("XY2 "<<x<<" "<<y<<" "<<radius);

            tf::Vector3 spiralPoint(x, y, 0);
            tf::Quaternion q = tf::createQuaternionFromRPY(rotation.getX(), rotation.getY(), rotation.getZ());
            spiralPoint = tf::quatRotate(q, spiralPoint);

            //			ROS_WARN_STREAM("SP XYZ "<<spiralPoint.getX()<<" "<<spiralPoint.getY()<<" "<<spiralPoint.getZ());

            geometry_msgs::Point spiralRotated;

            // Get workspace rotation only if not in plane Z
            //			if (planeFace < 3) {
            //				spiralRotated.x = spiralPoint.getX()*cos(this->rotation) + spiralPoint.getY()*sin(this->rotation);
            //				spiralRotated.y = spiralPoint.getY()*cos(this->rotation) - spiralPoint.getX()*sin(this->rotation);
            //				spiralRotated.z = spiralPoint.getZ();
            //			}
            //			else {
            spiralRotated.x = spiralPoint.getX();
            spiralRotated.y = spiralPoint.getY();
            spiralRotated.z = spiralPoint.getZ();
            //			}
            //			ROS_WARN_STREAM("XYZ "<<spiralRotated.x<<" "<<spiralRotated.y<<" "<<spiralRotated.z);

            TempGoalNode->x = coordBackup.x + spiralRotated.x;
            TempGoalNode->y = coordBackup.y + spiralRotated.y;
            TempGoalNode->z = coordBackup.z + spiralRotated.z;
            if(this->isGoalWithinStateSpace(TempGoalNode, margin)){
                //ROS_WARN("%f %f", angle, lastAngle);
                lastAngle = angle;
            }else{
                if(angle - lastAngle > 2*M_PI){

                    // Control 2 sides
                    if (signedPlane)
                    {
                        tries++;
                    }

                    // If all sides evaluated
                    if (tries == 3)
                    {
                        TempGoalNode = nullptr;
                        ROS_ERROR("GoalPoint could not be defined using spiral!");
                        break;
                    }

                    // Verify next plane to move
                    if((tries == 1) && signedPlane)
                    {
                        switch(planeFace)
                        {
                            case 2: {
                                planeFace = 1;
                                break;
                            }
                            case 1:
                            case 3: {
                                planeFace = 2;
                                break;
                            }
                            default:
                                break;
                        }
                    }
                    else if (signedPlane) {
                        switch(planeFace)
                        {
                            case 2: {
                                planeFace = 1 + 2*(firstPlane == 3);
                                break;
                            }
                            case 1:{ // 2 is never the last
                                planeFace = 3;
                                break;
                            }
                            default:
                                break;
                        }
                    }

                    // Adjust rotation
                    rotation[0] = 0;
                    rotation[1] = 0;
                    rotation[2] = 0;
                    rotation[planeFace-1] = 1;
                    rotation *= M_PI/2;

                    Node* offsetToTempGoal = new Node(0,0,0);
                    offsetToTempGoal->x = (planeFace==2)*this->_size.x*0.4;
                    offsetToTempGoal->y = (planeFace==1)*this->_size.y*0.4;
                    offsetToTempGoal->z = (planeFace==3)*this->_size.z*0.4;

                    ROS_WARN("OFFSET %f %f %f %f", offsetToTempGoal->x, offsetToTempGoal->y, offsetToTempGoal->z, this->rotation);
                    double originalX = offsetToTempGoal->x;
                    offsetToTempGoal->x = offsetToTempGoal->x*cos(this->rotation) + offsetToTempGoal->y*sin(this->rotation);
                    offsetToTempGoal->y = offsetToTempGoal->y*cos(this->rotation) - originalX*sin(this->rotation);
                    offsetToTempGoal->z = offsetToTempGoal->z;

                    ROS_WARN("OFFSET %f %f %f", offsetToTempGoal->x, offsetToTempGoal->y, offsetToTempGoal->z);

                    // CHANGE THIS TO CLOSER TO GOAL
                    if (signedPlane) {
                        TempGoalNode->x = this->_center.x - offsetToTempGoal->x;
                        TempGoalNode->y = this->_center.y - offsetToTempGoal->y;
                        TempGoalNode->z = this->_center.z - offsetToTempGoal->z;
                    }
                    else {
                        TempGoalNode->x = this->_center.x + offsetToTempGoal->x;
                        TempGoalNode->y = this->_center.y + offsetToTempGoal->y;
                        TempGoalNode->z = this->_center.z + offsetToTempGoal->z;
                    }

                    delete offsetToTempGoal;
                    coordBackup = TempGoalNode->getNodeAsPoint();

                    x = 0.0;
                    y = 0.0;
                    angle = 0.1;
                    radius = b * angle;
                    lastAngle = angle;

                    ROS_WARN("\n\n\nCHANGED PLANE To %d IN TRY %d SIGNED %d!!!!\n %f %f %f\n\n\n", planeFace, tries, signedPlane, TempGoalNode->x, TempGoalNode->y, TempGoalNode->z);

                    signedPlane = 1 - signedPlane;
                }
            }
        }
    }else{
        ROS_ERROR("GoalPoint not defined!");
    }

    if(TempGoalNode != nullptr)
        ROS_WARN("TEMPGOAL %f %f %f %d", TempGoalNode->x, TempGoalNode->y, TempGoalNode->z, this->isGoalWithinStateSpace(TempGoalNode, margin));

    return TempGoalNode;
}

geometry_msgs::Point StateSpace::getRotatedVectorToCenter(Node *node, double angle){
    geometry_msgs::Point v, o;
    v.x = node->x - this->_center.x;
    v.y = node->y - this->_center.y;
    v.z = node->z - this->_center.z;

    o.x = v.x*cos(angle) + v.y*sin(angle);
    o.y = v.y*cos(angle) - v.x*sin(angle);
    o.z = v.z;

    return o;
}

geometry_msgs::Point StateSpace::getGeometricCenter(){
    geometry_msgs::Point output;
    output.x = (this->min_x + this->max_x)/2;
    output.y = (this->min_y + this->max_y)/2;
    output.z = (this->min_z + this->max_z)/2;
    return output;
}

geometry_msgs::Point StateSpace::getCenter(){
    return this->_center;
}

geometry_msgs::Point StateSpace::getSize(){
    return this->_size;
}
