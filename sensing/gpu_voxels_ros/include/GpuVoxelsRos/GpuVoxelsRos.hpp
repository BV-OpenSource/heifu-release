#ifndef GPUVOXELSROS_HPP
#define GPUVOXELSROS_HPP

#include <cstdlib>
#include <signal.h>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <icl_core_config/Config.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <mutex>
#include <shared_mutex>

using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using namespace gpu_voxels::voxelmap;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxelmap::BitVectorVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;

class GPUVoxelsROS
{
    public:
        GPUVoxelsROS();
        GPUVoxelsROS(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);

        void Setup();
        void Run();
        void cbPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);
        void MoveMap(const Vector3f mapDisplacement);
        void cbCalib(const std_msgs::Empty);
        void cbCleanMap(const std_msgs::Empty);
    private:
        // ROS Handler
        ros::NodeHandle nh;
        ros::NodeHandle shm_nh;

        // Parameters
        double paramNodeRate;
        double paramVoxelSize;
        double paramErodeThreshold;
        double paramOccupancy;
        double paramWidth;
        double paramHeight;
        double paramHFOV;
        double paramVFOV;
        double paramMaxRange;
        double paramProbHit;
        double paramProbMiss;

        bool paramForceRaycast;

        int paramDensityThreshold;

        std::string paramPointCloudTopic;
        std::string paramCloudSourceFrame;
        std::string paramCloudTargetFrame;

        // Subscribers
        ros::Subscriber subPointCloud;
        ros::Subscriber subCalib;
        ros::Subscriber subCleanMap;

        // Publishers
        ros::Publisher pubMap;
        ros::Publisher pubMapOffset;
        ros::Publisher pubPCD;
        ros::Publisher pubMutex;

        // Transforms
        tf::TransformListener listener;
        tf::TransformBroadcaster broadcaster;

        // Variables
        std::string ns;

        shared_ptr<GpuVoxels> gvl;
        shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap;
        shared_ptr<ProbVoxelMap> erodeTempVoxmap1;
        shared_ptr<ProbVoxelMap> erodeTempVoxmap2;
        shared_ptr<BitVectorVoxelMap> rob_map;
        std::mutex mapAccessMutex;

        std_msgs::UInt64Ptr mapPub;
        std_msgs::UInt64Ptr mapMutex;
        geometry_msgs::PointStamped mapOffsetPub;
        Vector3ui map_dimensions;

        sensor_msgs::PointCloud2 pubPointCloud;

        std::string tfName;

        tf::StampedTransform brStTransform;
        tf::Transform brTransform;
        tf::Quaternion brQuaternion;

        bool new_data_received;
        PointCloud my_point_cloud;
        PointCloud emptyPointCloud;
        Matrix4f tf;

        Vector3f sensorPosition;
        Vector3f mapOffset;
        Vector3f mapRealDimensions;
        Vector3f mapDisplacement;
        double maxRange;
        double horizontalFOV;
        double verticalFOV;
        double sensorWidth;
        double sensorHeight;
        double hAngleStep;
        double vAngleStep;
        double hAngleOffset;
        double vAngleOffset;

        bool calibration;
};

#endif // GPUVOXELSROS_HPP
