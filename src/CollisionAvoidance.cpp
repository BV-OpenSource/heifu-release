#include "CollisionAvoidance/CollisionAvoidance.hpp"

CollisionAvoidance::CollisionAvoidance():nh("~")
{
    CollisionAvoidance(nh, nh);
}

CollisionAvoidance::CollisionAvoidance(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):nh(private_nh_),shm_nh(nh_){
    ns = ros::this_node::getNamespace();

    //	Subscribers
    subMapPtr           = shm_nh.subscribe(ns + "/GPU_Voxels/map", 1, &CollisionAvoidance::cbGetMapPtr, this);
    subMapOffset        = shm_nh.subscribe(ns + "/GPU_Voxels/offset", 1, &CollisionAvoidance::cbMapOffset, this);
    subMavrosVel        = nh.subscribe(ns + "/mavros/local_position/velocity_local", 1, &CollisionAvoidance::cbMavrosVel, this);
    subMavrosPose       = nh.subscribe(ns + "/mavros/local_position/pose", 1, &CollisionAvoidance::cbMavrosPose, this);
    subMavrosState      = nh.subscribe(ns + "/mavros/state", 1, &CollisionAvoidance::cbMavrosState, this);
    subSetpointPos      = shm_nh.subscribe(ns + "/waypointsManager/goalWaypoint", 1, &CollisionAvoidance::cbSetpointPos, this);
    subPlannersSetpoint = shm_nh.subscribe(ns + "/planners/local_position/setpoint", 1, &CollisionAvoidance::cbSetpointPos, this);

    subDebugCollision   = nh.subscribe(ns + "/collisionAvoidance/STOP", 1, &CollisionAvoidance::cbSendStopCommand, this);

    subMutexPtr = shm_nh.subscribe(ns + "/GPU_Voxels/MutexMap", 1, &CollisionAvoidance::cbGetMutexPtr, this);

    //	Publisher
    pubPause = nh.advertise<std_msgs::Empty>(ns + "/planners/collision/pause", 10);
    pubStart = nh.advertise<std_msgs::Empty>(ns + "/planners/collision/start", 10);
    pubMarker = nh.advertise<visualization_msgs::Marker>(ns + "/visualization_marker", 10);
    pubUpdateDistance = nh.advertise<std_msgs::Empty>(ns + "/planners/updateSafetyDistance", 10);
    pubCollisionDetected = nh.advertise<std_msgs::Empty>(ns + "/collisionAvoidance/emergencyStop", 10);

    pubBrakeMode = nh.advertise<std_msgs::Empty>(ns + "/mode_brake", 10);

    std_msgs::Empty msg;
    pubUpdateDistance.publish(msg);

    //	Parameters
    nh.param<double>("paramNodeRate",           paramNodeRate,           100.0);
    nh.param<double>("paramMinSafetyDistance",  paramMinSafetyDistance,    2.0);
    nh.param<double>("paramMaxSafetyDistance",  paramMaxSafetyDistance,    4.0);
    nh.param<double>("paramMaxVisionDistance",  paramMaxVisionDistance,   20.0);

    // Init robot position
    lastRobotPosition.pose.position.x = 0;
    lastRobotPosition.pose.position.y = 0;
    lastRobotPosition.pose.position.z = 0;

    robotPosition.pose.position.x = 0;
    robotPosition.pose.position.y = 0;
    robotPosition.pose.position.z = 10;

    robotWaypointVector.x = 0;
    robotWaypointVector.y = 0;
    robotWaypointVector.z = 0;

    // DEBUG
    obj.type = visualization_msgs::Marker::SPHERE;
    obj.header.frame_id = ns + "/odom";
    if(obj.header.frame_id[0] == '/'){
        obj.header.frame_id.erase(0,1);
    }
    obj.header.stamp = ros::Time::now();

    obj.ns = "markers";
    obj.id = 10;

    obj.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker
    obj.scale.x = obj.scale.y = obj.scale.z = 2*paramMinSafetyDistance;
    obj.pose.orientation.x = obj.pose.orientation.y = obj.pose.orientation.z = 0.0;
    obj.pose.orientation.w = 1.0;

    obj.color.r = 0.5f;
    obj.color.g = 0.5f;
    obj.color.b = 0.5f;
    obj.color.a = 0.0f;

    obj.lifetime = ros::Duration();

    State = "GUIDED";
    goalReceived = false;
}

void CollisionAvoidance::cbMavrosVel(const geometry_msgs::TwistStamped &msg){
    robotVelocityVector = msg.twist.linear;
}

Vector3ui CollisionAvoidance::convertPoint(const geometry_msgs::Point point){
    Vector3ui convertedPoint;
    convertedPoint.x = (point.x + center.x)/VoxelSize;
    convertedPoint.y = (point.y + center.y)/VoxelSize;
    convertedPoint.z = (point.z + center.z)/VoxelSize;
    return convertedPoint;
}

void CollisionAvoidance::cbMavrosState(const mavros_msgs::State &msg){
    State = msg.mode;
}

void CollisionAvoidance::cbSendStopCommand(const std_msgs::Empty){
    pubBrakeMode.publish(msgEmpty);
    pubCollisionDetected.publish(msgEmpty);
}

void CollisionAvoidance::cbMavrosPose(const geometry_msgs::PoseStamped &msg){

    robotPosition = msg;

    if(DistanceVoxmap != nullptr){
        double velXY = robotVelocityVector.x*robotVelocityVector.x + robotVelocityVector.y*robotVelocityVector.y;
        bool velocityImpact = velXY > 0.5;
        double velMod = sqrt(velXY + robotVelocityVector.z*robotVelocityVector.z);

        // Going vertical
        if(!velocityImpact && velMod > 0.5){
            ROS_INFO_THROTTLE(1.0, "Vertical Movement Detected!");
        }else{
            // Brake mode, stopped and with goal set
            if(State != "BRAKE" && velocityImpact){

                // NEEDED DUE TO DRONE POSITIONING UNCERTAINTIES
                robotWaypointVector.x = goalPosition.x - robotPosition.pose.position.x;
                robotWaypointVector.y = goalPosition.y - robotPosition.pose.position.y;
                robotWaypointVector.z = goalPosition.z - robotPosition.pose.position.z;

                if(!goalReceived){
                    goalPosition = robotPosition.pose.position;
                }

                if(velocityImpact && !goalReceived){
                    double velScale = 4.0;
                    ROS_WARN("UAV is moving but no setpoint was given.");
                    robotWaypointVector.x = robotVelocityVector.x * velScale;
                    robotWaypointVector.y = robotVelocityVector.y * velScale;
                    robotWaypointVector.z = robotVelocityVector.z * velScale;
                }

                double modPos = sqrt(robotWaypointVector.x*robotWaypointVector.x+robotWaypointVector.y*robotWaypointVector.y+robotWaypointVector.z*robotWaypointVector.z);

                // TAKE ADVANTAGE OF SENSOR RANGE
                if(modPos > paramMaxVisionDistance){
                    double tempScale = paramMaxVisionDistance/modPos;
                    robotWaypointVector.x *= tempScale;
                    robotWaypointVector.y *= tempScale;
                    robotWaypointVector.z *= tempScale;
                }

                modPos = sqrt(robotWaypointVector.x*robotWaypointVector.x+robotWaypointVector.y*robotWaypointVector.y+robotWaypointVector.z*robotWaypointVector.z);

                //		ROS_ERROR("MIN: %f ", std::min(modPos + paramSafetyDistance-2, 10.0));

                nextPoint = robotWaypointVector;

                double nextPointScale = sqrt(nextPoint.x*nextPoint.x + nextPoint.y*nextPoint.y + nextPoint.z*nextPoint.z);
                //        nextPointScale /= maxSphereDistance;

                // Normalize Sphere Direction Vector
                if(nextPointScale > 2*DBL_EPSILON){
                    nextPoint.x /= nextPointScale;
                    nextPoint.y /= nextPointScale;
                    nextPoint.z /= nextPointScale;
                }

                geometry_msgs::Point measurementPoint; // = robotPosition.pose.position;

                Vector3ui measurementVector;
                double lastSphere = 1;
                for (size_t i = 1; i < sphereDistance.size() || i < vect.size(); i++) {
                    if(sphereDistance.at(i) < nextPointScale){
                        lastSphere = i+1;
                    }else{
                        vect[i].color.a = 0.0f;
                    }
                }

                for (size_t i=0; i < lastSphere; i++) {
                    measurementPoint = robotPosition.pose.position;
                    measurementPoint.x += nextPoint.x * sphereDistance.at(i);
                    measurementPoint.y += nextPoint.y * sphereDistance.at(i);
                    measurementPoint.z += nextPoint.z * sphereDistance.at(i);

                    measurementVector = convertPoint(measurementPoint);

                    std::unique_lock<std::mutex> lock(*mapAccessMutex, std::defer_lock);
                    do { lock.try_lock(); } while(!lock.owns_lock());
                    //cudaMemcpy(&dv, (dvm_thrust_ptr+id).get(), sizeof(DistanceVoxel), cudaMemcpyDeviceToHost);
                    double metric_free_space = static_cast<double>(VoxelSize * sqrt(
                                                                       pbaDistanceVoxmap->getSquaredObstacleDistance(measurementVector)));
                    lock.unlock();

                    // ROS_ERROR("FREE SPACE: %f", metric_free_space);

                    if(metric_free_space < safetyRadius.at(i)){
                        if(State != "BRAKE"){
                            cbSendStopCommand(msgEmpty);

                            pubPause.publish(msgEmpty);

                            ROS_INFO("Obstacle %f %f %f", measurementPoint.x, measurementPoint.y, measurementPoint.z);
                            ROS_INFO_STREAM(" has a clearance of " << metric_free_space << "m so "<< metric_free_space + sphereDistance.at(i) << "m from vehicle at " << velMod << " m/s." );
                            goalPosition = robotPosition.pose.position;

                            ros::Duration(5.0).sleep();
                            //                    goalReceived = false;
                            break;
                        }
                        else{
                            ROS_WARN("DEBUG PRINT: brake detected!");
                        }
                    }else{
                        //				ROS_INFO_STREAM(" Voxel @ " << measurementPoint << " has a clearance of " << metric_free_space << "m." );
                    }

                    // Prevent out of index for rviz spheres
                    if(i < vect.size()){
                        vect[i].pose.position = measurementPoint;
                        vect[i].color.a = 0.5f;
                    }else {
                        vect.back().pose.position = measurementPoint;
                        vect.back().color.a = 0.5f;
                    }
                }
            }else{
                for (size_t i = 0; i < vect.size(); i++) {
                    vect[i].color.a = 0.0f;
                }
                goalReceived = false;
                //            if(goalReceived && pubStart.getNumSubscribers() > 0){
                //                ROS_INFO("Start Planners from Collision.");
                //                pubStart.publish(msgEmpty);
                //            }
            }
        }
    }else{
        ROS_WARN_THROTTLE(2.0, "CollisionAvoidance: DistanceVoxmap == nullptr");
    }

    for(auto obj:vect){
        pubMarker.publish(obj);
    }
}

void CollisionAvoidance::cbSetpointPos(const uav_msgs::UAVWaypoint &msg){
    goalPosition.x = msg.x;
    goalPosition.y = msg.y;
    goalPosition.z = msg.z;

    ROS_INFO("\tCollision SetPoint Pos %f %f %f", goalPosition.x, goalPosition.y, goalPosition.z);
    robotWaypointVector.x = goalPosition.x - robotPosition.pose.position.x;
    robotWaypointVector.y = goalPosition.y - robotPosition.pose.position.y;
    robotWaypointVector.z = goalPosition.z - robotPosition.pose.position.z;

    if(robotWaypointVector.x*robotWaypointVector.x > 0.01 || robotWaypointVector.y*robotWaypointVector.y > 0.01 || robotWaypointVector.z*robotWaypointVector.z > 0.01){
        goalReceived = true;
    }
}

void CollisionAvoidance::cbGetMapPtr(const std_msgs::UInt64 &msg){
    DistanceVoxmap = reinterpret_cast<gpu_voxels::DistanceVoxel*>(msg.data);
    //	std::vector< std::string> params;
    //	ros::param::getParamNames(params);
    //	for(std::string str:params){
    // TODO: check droneID
    ros::param::get(ns+"/gpu_voxels_ros_nodelet/paramVoxelSize", VoxelSize);
    std::vector<int> map_size(3);
    ros::param::get(ns+"/gpu_voxels_ros_nodelet/paramMapSize", map_size);
    mapDimensions.x = static_cast<uint32_t>(map_size[0]);
    mapDimensions.y = static_cast<uint32_t>(map_size[1]);
    mapDimensions.z = static_cast<uint32_t>(map_size[2]);
    //	}
    center = voxelmap::getVoxelCenter(VoxelSize, Vector3ui(mapDimensions.x/2, mapDimensions.y/2, mapDimensions.z/2));
    pbaDistanceVoxmap = new gpu_voxels::voxelmap::DistanceVoxelMap(DistanceVoxmap, mapDimensions, VoxelSize, MT_DISTANCE_VOXELMAP);

    thrust::device_ptr<DistanceVoxel> dvm_thrust_ptr(DistanceVoxmap);
    ROS_INFO_ONCE("Collision Avoidance Device Ptr: %x", dvm_thrust_ptr);

    ROS_INFO_STREAM("Vsize " << VoxelSize << " center " << center);
    double coneScale = (paramMinSafetyDistance - paramMaxSafetyDistance)/paramMaxVisionDistance;

    double cS2 = coneScale*coneScale;
    double v2 = VoxelSize*VoxelSize;

    double a0 = 4 * paramMaxSafetyDistance*paramMaxSafetyDistance * cS2;
    double a1 = 4 * paramMaxSafetyDistance * VoxelSize;
    double a3 = (VoxelSize - 2*paramMaxSafetyDistance) * coneScale;
    double a4 = (2 * cS2 + 2);

    double b0 = a0 + a1 - v2;

    double c0 = 2*paramMaxSafetyDistance*coneScale;
    double c1 = cS2 - 1;

    double x0 = (sqrt(b0) + a3) / a4;
    double d0 = - (2*x0 + c0) / c1;
    double r0 = d0 * coneScale + paramMaxSafetyDistance;

    //    sphereDistance.push_back(0.0);
    //    safetyRadius.push_back(paramMaxSafetyDistance);

    sphereDistance.push_back(paramMaxVisionDistance - d0);
    safetyRadius.push_back(r0);

    //    vect.push_back(obj);
    //    vect.at(0).scale.x = vect.at(0).scale.y = vect.at(0).scale.z = 2*r0;

    ROS_WARN_STREAM("x0: " << x0 << " d0: " << d0 << " r0: " << r0);

    for(size_t i=1; d0 < paramMaxVisionDistance; i++){
        double S3 = d0;
        x0 = (sqrt(4 * S3*S3 * cS2*cS2 + 8 * paramMaxSafetyDistance * S3 * coneScale*cS2 + b0 + 4 * VoxelSize * S3 * coneScale) + a3 + 2*S3) / a4;
        d0 = -(2*x0 + S3*cS2 + c0 - S3) / c1;
        r0 = d0 * coneScale + paramMaxSafetyDistance;

        sphereDistance.push_back(paramMaxVisionDistance - d0);
        safetyRadius.push_back(r0);
        obj.scale.x = obj.scale.y = obj.scale.z = 2*r0;
        obj.color.r = 1.0f*(i&1);
        obj.color.g = 1.0f*(i&2);
        obj.color.b = 1.0f*(i&4);
        obj.id = (i+1)*10;
        vect.push_back(obj);
        ROS_WARN_STREAM("x0: " << x0 << " d0: " << d0 << " r0: " << r0);
    }

    std::reverse(vect.begin(), vect.end() );
    std::reverse(safetyRadius.begin(), safetyRadius.end() );
    std::reverse(sphereDistance.begin(), sphereDistance.end() );

    subMapPtr.shutdown();
}

void CollisionAvoidance::cbGetMutexPtr(const std_msgs::UInt64 &msg)
{
    mapAccessMutex = reinterpret_cast<std::mutex*>(msg.data);
    subMutexPtr.shutdown();
}

void CollisionAvoidance::cbMapOffset(const geometry_msgs::PointStampedPtr msg){
    center.x = static_cast<float>(msg->point.x);
    center.y = static_cast<float>(msg->point.y);
    center.z = static_cast<float>(msg->point.z);
}

void CollisionAvoidance::Run(){
    ros::Rate go(paramNodeRate);

    while (ros::ok() ){
        ROS_INFO("Starting...");

        for(auto obj:vect){
            pubMarker.publish(obj);
        }

        //ros spins, force ROS frame to refresh/update once
        ros::spinOnce();

        go.sleep();
    }
}
