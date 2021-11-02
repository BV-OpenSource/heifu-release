#include "GpuVoxelsRos/GpuVoxelsRos.hpp"

GPUVoxelsROS::GPUVoxelsROS():nh("~"){
    GPUVoxelsROS(nh, nh);
}

GPUVoxelsROS::GPUVoxelsROS(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):nh(private_nh_),shm_nh(nh_){
    ns = ros::this_node::getNamespace();

    /* Parameters
     * paramNodeRate - Rate of ROS node in hz
     * paramVoxelSize - Side length of a voxel
     * paramDensityThreshold - Density filter threshold per voxel
     * paramErodeThreshold - Erode voxels with fewer occupied neighbors (percentage)
     * paramPointCloudTopic - Identifer of the pointcloud topic
     */
    nh.param<double>("paramNodeRate", paramNodeRate, 100.0);
    nh.param<double>("paramVoxelSize", paramVoxelSize, 0.01);
    nh.param<double>("paramErodeThreshold", paramErodeThreshold, 10);
    nh.param<double>("paramOccupancy", paramOccupancy, 0.5);
    nh.param<double>("paramWidth", paramWidth, 680);
    nh.param<double>("paramHeight", paramHeight, 480);
    nh.param<double>("paramHFOV", paramHFOV, 1.29);
    nh.param<double>("paramVFOV", paramVFOV, 1.08);
    nh.param<double>("paramMaxRange", paramMaxRange, 10);
    nh.param<double>("paramProbHit", paramProbHit, 0.50);
    nh.param<double>("paramProbMiss", paramProbMiss, 0.10);

    nh.param<bool>("paramForceRaycast", paramForceRaycast, false);

    nh.param<int>("paramDensityThreshold", paramDensityThreshold, 0);
    nh.param<std::string>("paramPointCloudTopic", paramPointCloudTopic, "");
    nh.param<std::string>("paramCloudSourceFrame", paramCloudSourceFrame, "");
    nh.param<std::string>("paramCloudTargetFrame", paramCloudTargetFrame, "");

    pubMap = shm_nh.advertise<std_msgs::UInt64>("GPU_Voxels/map", 10);
    pubMapOffset = shm_nh.advertise<geometry_msgs::PointStamped>("GPU_Voxels/offset", 10);
    pubPCD = shm_nh.advertise<sensor_msgs::PointCloud2>("GPU_Voxels/PCD",10);
    pubMutex = shm_nh.advertise<std_msgs::UInt64>("GPU_Voxels/MutexMap", 10);

    paramCloudSourceFrame = ns + paramCloudSourceFrame;
    paramCloudTargetFrame = ns + paramCloudTargetFrame;

    paramDensityThreshold *= 4*pow(paramVoxelSize,3);

    std::vector<int> map_size;
    //	nh.param<std::vector<int> >("paramMapSize", map_size, std::vector<int>(3, 512));
    nh.param<std::vector<int> >("paramMapSize", map_size, std::vector<int>(3, 256));

    map_dimensions.x = static_cast<uint32_t>(map_size[0]);
    map_dimensions.y = static_cast<uint32_t>(map_size[1]);
    map_dimensions.z = static_cast<uint32_t>(map_size[2]);

    mapRealDimensions.x = map_dimensions.x * paramVoxelSize;
    mapRealDimensions.y = map_dimensions.y * paramVoxelSize;
    mapRealDimensions.z = map_dimensions.z * paramVoxelSize;

    // Subscribers
    subPointCloud = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(ns + paramPointCloudTopic, 1, &GPUVoxelsROS::cbPointCloud, this);
    ROS_INFO_STREAM("DistanceROSDemo start. Point-cloud topic: " << ns + paramPointCloudTopic);
    subCalib = nh.subscribe<std_msgs::Empty >(ns + "/GPU_Voxels/pcdCalib", 1, &GPUVoxelsROS::cbCalib, this);
    subCleanMap = nh.subscribe<std_msgs::Empty >(ns + "/GPU_Voxels/cleanMap", 1, &GPUVoxelsROS::cbCleanMap, this);
}

void ctrlchandler(int){
    ROS_INFO("CTRL+C");
    exit(EXIT_SUCCESS);
}

void killhandler(int){
    ROS_INFO("SIGKILL");
    exit(EXIT_SUCCESS);
}

void GPUVoxelsROS::Setup(){
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    // Generate a GPU-Voxels instance:
    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(map_dimensions.x, map_dimensions.y, map_dimensions.z, paramVoxelSize);

    //PBA
    gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    pbaDistanceVoxmap = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmap"));

    thrust::device_ptr<DistanceVoxel> dvm_thrust_ptr(pbaDistanceVoxmap->getDeviceDataPtr());
    ROS_INFO_ONCE("GPU Device Ptr: %x", dvm_thrust_ptr);

    gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap1");
    erodeTempVoxmap1 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap1"));
    gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap2");
    erodeTempVoxmap2 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap2"));

    std::cout << "Remove voxels containing less points than: " << paramDensityThreshold << std::endl;
    std::cout << "Erode voxels with neighborhood occupancy ratio less or equal to: " << paramErodeThreshold << std::endl;

    new_data_received = false; // call visualize on the first iteration

    gvl->addMap(MT_BITVECTOR_VOXELMAP, "pbaRobotMap");
    rob_map = dynamic_pointer_cast<BitVectorVoxelMap>(gvl->getMap("pbaRobotMap"));

    maxRange = paramMaxRange;
    horizontalFOV = paramHFOV;
    sensorWidth = paramWidth;
    sensorHeight = paramHeight;
    verticalFOV = paramVFOV;//sensorHeight*horizontalFOV/sensorWidth; //0.994;
    //ROS_INFO("Resulting VFOV %f", verticalFOV);
    hAngleOffset = -maxRange/tan(M_PI/2-horizontalFOV/2);
    vAngleOffset = maxRange/tan(M_PI/2-verticalFOV/2);
    hAngleStep = hAngleOffset/(sensorWidth/2);
    vAngleStep = vAngleOffset/(sensorHeight/2);

    mapOffset = mapRealDimensions/2;

    mapOffsetPub.point.x = mapOffset.x;
    mapOffsetPub.point.y = mapOffset.y;
    mapOffsetPub.point.z = mapOffset.z;

    mapPub = std_msgs::UInt64Ptr(new std_msgs::UInt64);
    mapMutex = std_msgs::UInt64Ptr(new std_msgs::UInt64);

    brQuaternion.setRPY(0, 0, 0);
    tfName = ns + "/gpu_voxels";
    if(tfName[0] == '/'){
        tfName.erase(0,1);
    }
    pubPointCloud.header.frame_id = tfName;

    brStTransform = tf::StampedTransform();
    brStTransform.frame_id_ = paramCloudTargetFrame;
    brStTransform.child_frame_id_ = pubPointCloud.header.frame_id;
    brTransform.setRotation(brQuaternion);

    calibration = false;
}

void GPUVoxelsROS::cbCalib(const std_msgs::Empty ){
    calibration = true;
}

void GPUVoxelsROS::cbCleanMap(const std_msgs::Empty ){
    std::unique_lock<std::mutex> lock(mapAccessMutex, std::defer_lock);

    do {
        lock.try_lock();
    }while(!lock.owns_lock());

    erodeTempVoxmap1->clearMap();

    lock.unlock();
}

/**
 * Receives new pointcloud and transforms from camera to world coordinates
 *
 * NOTE: could also use msg->frame_id to derive the transform to world coordinates through ROS
 */
void GPUVoxelsROS::cbPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    std::vector<Vector3f> point_data;
    point_data.reserve(msg->points.size());

    std::vector<Vector3f> emptyData;

    uint32_t k=0;
    uint32_t l=0;
    if(msg->isOrganized()){
        emptyData.reserve(msg->points.size());
        for (uint32_t i = 0; i < msg->width; i++){
            for (uint32_t j = 0; j < msg->height; j++){
                if(isnan(msg->at(i,j).x) || isnan(msg->at(i,j).y) || isnan(msg->at(i,j).z)){
                    if(isnan(msg->at(i,j).y) && (!isnan(msg->at(i,j).x) && !isnan(msg->at(i,j).z)))
                        ROS_ERROR("\n\n\nNAN VALUES FOUND1 %f %f %f\n\n\n", msg->at(i,j).x, msg->at(i,j).y, msg->at(i,j).z);
                    emptyData.push_back(Vector3f(hAngleOffset - i*hAngleStep, vAngleOffset - j*vAngleStep, maxRange+1));
                    l++;
                }else{
                    if(calibration){
                        horizontalFOV = std::max(double(atan2(msg->at(i,j).x, msg->at(i,j).z))*2, horizontalFOV);
                        verticalFOV = std::max(double(atan2(msg->at(i,j).y, sqrt(pow(msg->at(i,j).z,2)+pow(msg->at(i,j).x,2))))*2, verticalFOV);
                    }
                    point_data.push_back(Vector3f(msg->at(i,j).x, msg->at(i,j).y, msg->at(i,j).z));
                    k++;
                }
            }
        }
    }else{
        ROS_WARN_ONCE("PointCloud is not dense!");
        for (uint32_t i = 0; i < msg->points.size(); i++)
        {
            if(!isnan(msg->points[i].x) && isnan(msg->points[i].y) && !isnan(msg->points[i].z))
                ROS_ERROR("\n\n\nNAN VALUES FOUND2 %f %f %f\n\n\n", msg->points[i].x, msg->points[i].y, msg->points[i].z);

            if(isnan(msg->points[i].x) || isnan(msg->points[i].x) || isnan(msg->points[i].z)){
                ROS_ERROR("NAN VALUES FOUND");
                continue;
            }else{
                if(calibration){
                    horizontalFOV = std::max(double(atan2(msg->points[i].x, msg->points[i].z))*2, horizontalFOV);
                    verticalFOV = std::max(double(atan2(msg->points[i].y, sqrt(pow(msg->points[i].z,2)+pow(msg->points[i].x,2))))*2, verticalFOV);
                }
                point_data.push_back(Vector3f(msg->points[i].x, msg->points[i].y, msg->points[i].z));
                k++;
            }
        }
        if(paramForceRaycast){
            emptyData.reserve(sensorWidth*sensorHeight);
            for (uint32_t i = 0; i < sensorWidth; i++){
                for (uint32_t j = 0; j < sensorHeight; j++){
                    emptyData.push_back(Vector3f(hAngleOffset - i*hAngleStep, vAngleOffset - j*vAngleStep, maxRange+1));
                    l++;
                }
            }
        }
    }

    if(calibration){
        ROS_INFO("Calibration %f %f %f", maxRange, horizontalFOV, verticalFOV);
        hAngleOffset = -maxRange/tan(M_PI/2-horizontalFOV/2);
        vAngleOffset = maxRange/tan(M_PI/2-verticalFOV/2);
        hAngleStep = hAngleOffset/(sensorWidth/2);
        vAngleStep = vAngleOffset/(sensorHeight/2);
    }
    calibration = false;

    point_data.resize(k+1);
    emptyData.resize(l+1);

    //my_point_cloud.add(point_data);
    my_point_cloud.update(point_data);
    emptyPointCloud.update(emptyData);

    // transform new pointcloud to world coordinates
    tf::StampedTransform transform;
    try{
        listener.waitForTransform(paramCloudTargetFrame, paramCloudSourceFrame, ros::Time(0), ros::Duration(2.0));
        listener.lookupTransform(paramCloudTargetFrame, paramCloudSourceFrame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    tf::Quaternion q = transform.getRotation();

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf::Vector3 t = transform.getOrigin();

    sensorPosition = Vector3f(t.getX(), t.getY(), t.getZ());
    Vector3f dPosition = (sensorPosition + mapOffset - mapRealDimensions/2);

    bool new_offset = false;
    if(fabs(dPosition.x) > 0.3*mapRealDimensions.x){
        new_offset = true;
    }
    if(fabs(dPosition.y) > 0.3*mapRealDimensions.y){
        new_offset = true;
    }
    if(fabs(dPosition.z) > 0.3*mapRealDimensions.z){
        new_offset = true;
    }

    if (new_offset)
    {
        MoveMap(Vector3f(-dPosition.x * (fabs(dPosition.x) > 0.3*mapRealDimensions.x),
                         -dPosition.y * (fabs(dPosition.y) > 0.3*mapRealDimensions.y),
                         -dPosition.z * (fabs(dPosition.z) > 0.3*mapRealDimensions.z)));

        mapOffsetPub.point.x = mapOffset.x;
        mapOffsetPub.point.y = mapOffset.y;
        mapOffsetPub.point.z = mapOffset.z;
        ROS_WARN_STREAM("GPU map moved " << mapOffset);
    }

    pubMapOffset.publish(mapOffsetPub);

    sensorPosition += mapOffset;

    tf = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(roll, pitch, yaw), sensorPosition);

    my_point_cloud.transformSelf(&tf);
    emptyPointCloud.transformSelf(&tf);

    new_data_received = true;

    Run();
}

void GPUVoxelsROS::MoveMap(const Vector3f mapDisplacement){
    erodeTempVoxmap2->clone(*erodeTempVoxmap1);
    erodeTempVoxmap1->clearMap();
    erodeTempVoxmap2->moveInto(*erodeTempVoxmap1, mapDisplacement);
    mapOffset += mapDisplacement;
    new_data_received = false;
}

void GPUVoxelsROS::Run(){
    // visualize new pointcloud if there is new data
    if (new_data_received)
    {
        new_data_received = false;

        std::unique_lock<std::mutex> lock(mapAccessMutex, std::defer_lock);

        do {
            lock.try_lock();
        }while(!lock.owns_lock());

        pbaDistanceVoxmap->clearMap();
        //			erodeTempVoxmap1->clearMap();
        erodeTempVoxmap2->clearMap();

        erodeTempVoxmap1->insertSensorData(emptyPointCloud, sensorPosition, true, true, eBVM_OCCUPIED, Probability(-paramProbMiss*127), rob_map->getDeviceDataPtr());
        erodeTempVoxmap1->insertSensorData(my_point_cloud,  sensorPosition, true, true, eBVM_OCCUPIED, Probability(paramProbHit*127), rob_map->getDeviceDataPtr());

        if (paramErodeThreshold > 0){
            erodeTempVoxmap1->erodeInto(*erodeTempVoxmap2, paramErodeThreshold);
        }else{
            erodeTempVoxmap1->erodeLonelyInto(*erodeTempVoxmap2); //erode only "lonely voxels" without occupied neighbors
        }

        if (pubPCD.getNumSubscribers() > 0)
        {
            brTransform.setOrigin( tf::Vector3(-mapOffset.x, -mapOffset.y, -mapOffset.z) );
            brStTransform.stamp_ = ros::Time::now();
            brStTransform.setData(brTransform);
            broadcaster.sendTransform(brStTransform);

            erodeTempVoxmap2->publishPointcloud(&pubPointCloud, paramOccupancy);

            // Frame ID is set by GPU_Voxels. Reset to wanted frame.
            pubPointCloud.header.frame_id = tfName;

            pubPointCloud.header.stamp = brStTransform.stamp_;
            pubPCD.publish(pubPointCloud);
        }

        pbaDistanceVoxmap->mergeOccupied(erodeTempVoxmap2, Vector3ui(), paramOccupancy);

        // Calculate the distance map:
        pbaDistanceVoxmap->parallelBanding3D();

        lock.unlock();

        if (pubMap.getNumSubscribers() > 0)
        {
            mapPub->data = reinterpret_cast<std_msgs::UInt64::_data_type>(pbaDistanceVoxmap->getDeviceDataPtr());
            pubMap.publish(mapPub);
        }

        if (pubMutex.getNumSubscribers() > 0)
        {
            mapMutex->data = reinterpret_cast<std_msgs::UInt64::_data_type>(&mapAccessMutex);
            pubMutex.publish(mapMutex);
        }
    }
}

