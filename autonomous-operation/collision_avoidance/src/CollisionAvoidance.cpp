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
    subSetpointPos      = shm_nh.subscribe(ns + "/waypointsManager/goalWaypoint", 1, &CollisionAvoidance::cbSetpointPos, this);
    subPlannersSetpoint = shm_nh.subscribe(ns + "/planners/local_position/setpoint", 1, &CollisionAvoidance::cbSetpointPos, this);

    subMutexPtr = shm_nh.subscribe(ns + "/GPU_Voxels/MutexMap", 1, &CollisionAvoidance::cbGetMutexPtr, this);

    //	Publisher
    pubMarker = nh.advertise<visualization_msgs::Marker>(ns + "/visualization_marker", 10);
    pubPause = nh.advertise<std_msgs::Empty>(ns + "/planners/collision/pause", 10);
    pubStart = nh.advertise<std_msgs::Empty>(ns + "/planners/collision/start", 10);
    pubUpdateDistance = nh.advertise<std_msgs::Empty>(ns + "/planners/updateSafetyDistance", 10);
    pubSetpointVel = nh.advertise<geometry_msgs::TwistStamped>(ns + "/collisionAvoidance/emergencyStop", 10);

    std_msgs::Empty msg;
    pubUpdateDistance.publish(msg);

    //	Parameters
    nh.param<double>("paramNodeRate",       paramNodeRate,          100.0);
    nh.param<double>("paramSafetyDistance", paramSafetyDistance,    2.0);

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
    visualization_msgs::Marker obj;
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
    obj.scale.x = obj.scale.y = obj.scale.z = 2*paramSafetyDistance;
    obj.pose.orientation.x = obj.pose.orientation.y = obj.pose.orientation.z = 0.0;
    obj.pose.orientation.w = 1.0;

    obj.color.r = 1.0f;
    obj.color.g = 0.0f;
    obj.color.b = 0.0f;
    obj.color.a = 0.5f;

    obj.lifetime = ros::Duration();

    vect.push_back(obj);
    obj.color.r = 1.0f;
    obj.color.g = 1.0f;
    obj.color.b = 0.0f;
    obj.id = 20;
    vect.push_back(obj);
    obj.color.r = 0.0f;
    obj.color.g = 1.0f;
    obj.color.b = 1.0f;
    obj.id = 30;
    vect.push_back(obj);
    obj.color.r = 0.0f;
    obj.color.g = 0.0f;
    obj.color.b = 1.0f;
    obj.id = 40;
    vect.push_back(obj);
    obj.color.r = 1.0f;
    obj.color.g = 0.0f;
    obj.color.b = 1.0f;
    obj.id = 50;
    vect.push_back(obj);

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

void CollisionAvoidance::cbMavrosPose(const geometry_msgs::PoseStamped &msg){
    bool velocityImpact = (robotVelocityVector.x*robotVelocityVector.x + robotVelocityVector.y*robotVelocityVector.y + robotVelocityVector.z*robotVelocityVector.z)>0.25;;

    robotPosition = msg;

    if(DistanceVoxmap != nullptr){

        // Brake mode, stopped and with goal set
        if(State != "GUIDED" && !velocityImpact && goalReceived){
            State = "GUIDED";

            ROS_INFO("Start Planners from Collision.");
            pubStart.publish(msgEmpty);

            return;
        }

        // NEEDED DUE TO DRONE POSITIONING UNCERTAINTIES
        robotWaypointVector.x = goalPosition.x - robotPosition.pose.position.x;
        robotWaypointVector.y = goalPosition.y - robotPosition.pose.position.y;
        robotWaypointVector.z = goalPosition.z - robotPosition.pose.position.z;

        if(!goalReceived){
            goalPosition = robotPosition.pose.position;
        }

        if(velocityImpact && !goalReceived){
            ROS_WARN("UAV is moving but no setpoint was given.");

            robotWaypointVector.x = robotVelocityVector.x*5/velocityImpact;
            robotWaypointVector.y = robotVelocityVector.y*5/velocityImpact;
            robotWaypointVector.z = robotVelocityVector.z*5/velocityImpact;
        }

        double modPos = sqrt(robotWaypointVector.x*robotWaypointVector.x+robotWaypointVector.y*robotWaypointVector.y+robotWaypointVector.z*robotWaypointVector.z);
        if(modPos > 10.0){
            robotWaypointVector.x /= modPos;
            robotWaypointVector.y /= modPos;
            robotWaypointVector.z /= modPos;
            robotWaypointVector.x *= 10;
            robotWaypointVector.y *= 10;
            robotWaypointVector.z *= 10;
        }

        // TAKE ADVANTAGE OF SENSOR RANGE (USE 10 meters or distance to WP, the closest)

        //		ROS_ERROR("MIN: %f ", std::min(modPos + paramSafetyDistance-2, 10.0));

        nextPoint = robotWaypointVector;

        double nextPointScale = sqrt(nextPoint.x*nextPoint.x + nextPoint.y*nextPoint.y + nextPoint.z*nextPoint.z);
        nextPointScale /= maxSphereDistance;

        if(nextPointScale > 2*DBL_EPSILON){
            nextPoint.x /= nextPointScale;
            nextPoint.y /= nextPointScale;
            nextPoint.z /= nextPointScale;
        }

        geometry_msgs::Point measurementPoint = robotPosition.pose.position;

        geometry_msgs::TwistStamped vel_setpoint;

        Vector3ui measurementVector;
        for (size_t i = nextPointScale+1; i<vect.size(); i++) {
            vect[i].color.a = 0.0f;
        }

        for (size_t i=0; i < static_cast<size_t>(nextPointScale); i++) {
            measurementPoint.x += nextPoint.x;
            measurementPoint.y += nextPoint.y;
            measurementPoint.z += nextPoint.z;

            // Prevent out of index for rviz spheres
            if(i < vect.size()){
                vect[i].pose.position = measurementPoint;
                vect[i].color.a = 0.5f;
            }else {
                vect.back().pose.position = measurementPoint;
                vect.back().color.a = 0.5f;
            }

            measurementVector = convertPoint(measurementPoint);

            for(auto obj:vect){
                pubMarker.publish(obj);
            }

            std::unique_lock<std::mutex> lock(*mapAccessMutex, std::defer_lock);
            do { lock.try_lock(); } while(!lock.owns_lock());
            //cudaMemcpy(&dv, (dvm_thrust_ptr+id).get(), sizeof(DistanceVoxel), cudaMemcpyDeviceToHost);
            double metric_free_space = static_cast<double>(VoxelSize * sqrt(
                                                               pbaDistanceVoxmap->getSquaredObstacleDistance(measurementVector)));
            lock.unlock();

            //			ROS_ERROR("FREE SPACE: %f", metric_free_space);

            if(metric_free_space < paramSafetyDistance){
                if(State != "BRAKE"){
                    //					goalPosition = robotPosition.pose.position;

                    vel_setpoint.twist.linear.x = 0.0;
                    vel_setpoint.twist.linear.y = 0.0;
                    vel_setpoint.twist.linear.z = 0.0;

                    vel_setpoint.twist.angular.x = 0.0;
                    vel_setpoint.twist.angular.y = 0.0;
                    vel_setpoint.twist.angular.z = 0.0;

                    vel_setpoint.header.stamp = ros::Time::now();
                    pubSetpointVel.publish(vel_setpoint);

                    std_msgs::Empty msgEmpty;
                    pubPause.publish(msgEmpty);

                    State = "BRAKE";
                    ROS_INFO("Obstacle %f %f %f", measurementPoint.x, measurementPoint.y, measurementPoint.z);
                    ROS_INFO_STREAM(" has a clearance of " << metric_free_space << "m so "<< metric_free_space+maxSphereDistance*double(i) << "m from vehicle." );
                    goalPosition = robotPosition.pose.position;

                    uav_msgs::UAVWaypoint temp;
                    temp.x = goalPosition.x;
                    temp.y = goalPosition.y;
                    temp.z = goalPosition.z;
                    cbSetpointPos(temp);

                    //                    goalReceived = false;
                    break;
                }
            }else{
                //				ROS_INFO_STREAM(" Voxel @ " << measurementPoint << " has a clearance of " << metric_free_space << "m." );
            }
        }
    }else{
        ROS_WARN("DistanceVoxmap == nullptr");
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
        goalReceived=true;
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
    mapDimensions.x = map_size[0];
    mapDimensions.y = map_size[1];
    mapDimensions.z = map_size[2];
    //	}
    center = voxelmap::getVoxelCenter(VoxelSize, Vector3ui(mapDimensions.x/2, mapDimensions.y/2, mapDimensions.z/2));
    pbaDistanceVoxmap = new gpu_voxels::voxelmap::DistanceVoxelMap(DistanceVoxmap, mapDimensions, VoxelSize, MT_DISTANCE_VOXELMAP);

    thrust::device_ptr<DistanceVoxel> dvm_thrust_ptr(DistanceVoxmap);
    ROS_INFO_ONCE("Collision Avoidance Device Ptr: %x", dvm_thrust_ptr);

    ROS_INFO_STREAM("Vsize " << VoxelSize << " center " << center);
    maxSphereDistance = 2*sqrt(paramSafetyDistance*VoxelSize - VoxelSize*VoxelSize/4.0);
    subMapPtr.shutdown();
}

void CollisionAvoidance::cbGetMutexPtr(const std_msgs::UInt64 &msg)
{
    mapAccessMutex = reinterpret_cast<std::mutex*>(msg.data);
    subMutexPtr.shutdown();
}

void CollisionAvoidance::cbMapOffset(const geometry_msgs::PointStampedPtr msg){
    center.x = msg->point.x;
    center.y = msg->point.y;
    center.z = msg->point.z;
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
