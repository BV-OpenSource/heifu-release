#ifndef COLLISIONAVOIDANCE_HPP
#define COLLISIONAVOIDANCE_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <uav_msgs/UAVWaypoint.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include "visualization_msgs/Marker.h"
#include <std_msgs/UInt64.h>

#include <gpu_voxels/GpuVoxels.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <mutex>

class CollisionAvoidance
{
    public:
        CollisionAvoidance();
        CollisionAvoidance(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);

        void Run();

        void cbMavrosPose(const geometry_msgs::PoseStamped &msg);
        void cbMavrosVel(const geometry_msgs::TwistStamped &msg);
        void cbSetpointPos(const uav_msgs::UAVWaypoint &msg);
        void cbGetMapPtr(const std_msgs::UInt64 &msg);
        void cbMapOffset(const geometry_msgs::PointStampedPtr msg);

        Vector3ui convertPoint(const geometry_msgs::Point point);

        void cbGetMutexPtr(const std_msgs::UInt64 &msg);
    private:
        ros::NodeHandle nh;
        ros::NodeHandle shm_nh;

        // Subscribers
        ros::Subscriber subMapPtr;
        ros::Subscriber subMutexPtr;
        ros::Subscriber subMapOffset;
        ros::Subscriber subMavrosVel;
        ros::Subscriber subMavrosPose;
        ros::Subscriber subSetpointPos;
        ros::Subscriber subPlannersSetpoint;

        // Publishers
        ros::Publisher pubPause;
        ros::Publisher pubStart;
        ros::Publisher pubUpdateDistance;
        ros::Publisher pubSetpointVel;
        ros::Publisher pubMarker;

        // Parameters
        double paramNodeRate;
        double paramSafetyDistance;

        // Variables
        geometry_msgs::PoseStamped robotPosition;
        geometry_msgs::PoseStamped lastRobotPosition;
        geometry_msgs::Point robotWaypointVector;
        geometry_msgs::Point goalPosition;
        geometry_msgs::Point nextPoint;
        geometry_msgs::Vector3 robotVelocityVector;
        std_msgs::Empty msgEmpty;

        double maxSphereDistance;

        std::string State;
        std::string ns;

        // Debug
        std::vector<visualization_msgs::Marker> vect;

        gpu_voxels::DistanceVoxel* DistanceVoxmap = nullptr;
        gpu_voxels::voxelmap::DistanceVoxelMap *pbaDistanceVoxmap;
        double VoxelSize;
        gpu_voxels::Vector3ui mapDimensions;

        Vector3f center;

        bool goalReceived;

        std::mutex *mapAccessMutex;
};

#endif // COLLISIONAVOIDANCE_HPP
