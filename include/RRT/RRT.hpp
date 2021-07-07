#ifndef RRT_HPP
#define RRT_HPP

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <Planner/Planner.hpp>
#include <ros/ros.h>
#include "RRT/RRT_tree.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mutex>

namespace RRT3D
{
    class RRT : public Planner
    {
        public:
            RRT();
            RRT(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
            virtual ~RRT (){};

            void Run();
            void Setup();

            void cbGoalPosition(const geometry_msgs::Pose &msg);
            void cbMavrosPose(const geometry_msgs::PoseStamped &msg);

            void cbStop(const std_msgs::Empty);
            void cbPause(const std_msgs::Empty);
            void cbStart(const std_msgs::Empty);
            void cbWPReached(const std_msgs::Empty);
            void cbGetMapPtr(const std_msgs::UInt64 &msg);
            void cbCollisionStarting(const std_msgs::Empty);
            void cbCollisionPause(const std_msgs::Empty msg);
            void cbCollisionStart(const std_msgs::Empty msg);
            void cbMapOffset(const geometry_msgs::PointStampedPtr msg);

            void cbGetMutexPtr(const std_msgs::UInt64 &msg);

            void cbNodeletLoop(const std_msgs::Empty);

            bool AddNode(Node *randomNode, Node *closestNode);
            void GetGoalWithinSS();

        private:

            // ROS Handler
            ros::NodeHandle n;
            ros::NodeHandle shm_nh;

            void find_goal();

            // Parameters
            bool paramBiasedRRT;

            double paramRRTStep;
            double paramBiasedRRTProb;

            // Visual objects
            visualization_msgs::Marker goal;

            //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
            visualization_msgs::Marker vertices, edges, sp_edges, workSpace;

            // Variables
            bool goalFound;
            bool pathFound;
            bool TempGoalReached;
            bool AddDroneNode;
            bool TempGoalSet;
            bool IgnoreSafety;
            bool rotationSent;

            double minDistance;
            double lastRotation;
            double cntRotation;

            Node *goalNode;
            Node *TempGoalNode;

            StateSpace *stateSpace = nullptr;

            RRT_Tree *rrt;

            std::vector<Node *> roughShortestPath;
            std::vector<Node *> robotPath;
            std::vector<Node *> smoothenedRobotPath;

            geometry_msgs::Point coordBackup;

            uav_msgs::UAVWaypoint goalSetpoint;

            Obstacle *ObstaclePtr = nullptr;

            uint32_t nodesInserted;
    };
}

#endif // RRT_HPP
