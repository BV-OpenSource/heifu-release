#include "Planner/Obstacle.hpp"

Obstacle::Obstacle(uint64_t address){
    deviceDistanceVoxel = reinterpret_cast<gpu_voxels::DistanceVoxel*>(address);
    std::string ns = ros::this_node::getNamespace();
    ros::param::get(ns+"/gpu_voxels_ros_nodelet/paramVoxelSize", VoxelSize);
    std::vector<int> map_size(3);
    ros::param::get(ns+"/gpu_voxels_ros_nodelet/paramMapSize", map_size);
    ros::param::get(ns+"/Planners_Manager_nodelet/paramRobotSize", robotSize);
    updateSafetyDistance(*std::max_element(robotSize.begin(), robotSize.end()));
    mapDimensions.x = uint(map_size[0]);
    mapDimensions.y = uint(map_size[1]);
    mapDimensions.z = uint(map_size[2]);
    pbaDistanceVoxmap = new DistanceVoxelMap(deviceDistanceVoxel, mapDimensions, float(VoxelSize), MT_DISTANCE_VOXELMAP);
}

bool Obstacle::isWithinObstacle(Node *node, bool ignoreSafety) {
    Vector3f temp(node->x, node->y, node->z);
    temp+=mapCenter;
    temp/=VoxelSize;

    Vector3ui measurementPoint(uint(temp.x), uint(temp.y), uint(temp.z));

    std::unique_lock<std::mutex> lock(*mapAccessMutex, std::defer_lock);
    do {
        lock.try_lock();
    }while(!lock.owns_lock());
    float metric_free_space = pbaDistanceVoxmap->getSquaredObstacleDistance(measurementPoint);
    //  ROS_INFO("PROTECT ACCESS %d", lock.owns_lock());
    lock.unlock();

    if(ignoreSafety){
        return (metric_free_space < VoxelSize);
    }else{
        return (metric_free_space < safetyDistance);
    }
}

bool Obstacle::edgeWithinObstacle(Node *node1, Node *node2, bool ignoreSafety) {
    float dx = node2->x - node1->x;
    float dy = node2->y - node1->y;
    float dz = node2->z - node1->z;

    float VectMag = sqrt(dx*dx+dy*dy+dz*dz)*2;
    float EpsilonCoef = VoxelSize/VectMag;

    Node *temp = new Node(node1->x, node1->y, node1->z);

    std::vector<uint> indices;
    if(uint(VectMag)>1){
        indices.reserve(uint(VectMag)-1);
    }
    Vector3ui measurementPoint;
    uint id;

    while(!temp->closeTo(node2, VoxelSize)){
        Vector3f tempVect(temp->x, temp->y, temp->z);
        tempVect+=mapCenter;
        tempVect/=VoxelSize;

        measurementPoint.x = uint(tempVect.x);
        measurementPoint.y = uint(tempVect.y);
        measurementPoint.z = uint(tempVect.z);

        id = voxelmap::getVoxelIndexUnsigned(mapDimensions, measurementPoint);
        indices.push_back(id);

        temp->x += dx*EpsilonCoef;
        temp->y += dy*EpsilonCoef;
        temp->z += dz*EpsilonCoef;
    }

    if(indices.size()>0){
        std::vector<int> distances(indices.size());

        std::unique_lock<std::mutex> lock(*mapAccessMutex, std::defer_lock);
        do {
            lock.try_lock();
        }while(!lock.owns_lock());
        pbaDistanceVoxmap->getSquaredDistancesToHost(indices, distances);
        lock.unlock();

        float distanceTreshold = ignoreSafety?1.0:safetyDistance;
        for(size_t i=0; i<indices.size(); i++){
            // Ignore safety only for nodes closer to node1 (normally the robot)
            if(ignoreSafety && distances[i]>safetyDistance){
                ignoreSafety = false;
                distanceTreshold = safetyDistance;
            }
            if(distances[i] <= distanceTreshold){
                return true;
            }
        }
        // If full path is unsafe
        if(ignoreSafety){
            return true;
        }
    }
    return false;
}

void Obstacle::updateMapCenter(Vector3f mapCenterMsg){
    mapCenter = mapCenterMsg;
}

void Obstacle::updateSafetyDistance(float newDistance){
    safetyDistance = newDistance+0.5;
    safetyDistance *= safetyDistance/(VoxelSize*VoxelSize);
}
