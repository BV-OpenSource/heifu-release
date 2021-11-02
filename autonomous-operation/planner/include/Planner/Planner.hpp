#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_msgs/UAVWaypoint.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Planner/StateSpace.hpp>
#include <std_msgs/UInt64.h>

class Planner
{
    public:
        Planner();
        Planner(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);

        virtual void cbStop(const std_msgs::Empty){ROS_INFO("Stop");}
        virtual void cbPause(const std_msgs::Empty){ROS_INFO("Pause");}
        virtual	void cbStart(const std_msgs::Empty){ROS_INFO("Start");}
        virtual void cbCollisionPause(const std_msgs::Empty){ROS_INFO("Collision Pause");}
        virtual	void cbCollisionStart(const std_msgs::Empty){ROS_INFO("Collision Start");}
        virtual void cbNodeletLoop(const std_msgs::Empty){}

        virtual void cbWPReached(const std_msgs::Empty){ROS_INFO("Planners Reached Position cb");}
        virtual void cbGoalPosition(const geometry_msgs::Pose &msg){}
        virtual void cbMavrosPose(const geometry_msgs::PoseStamped &msg){}
        virtual void cbOctoMap(const visualization_msgs::MarkerArray &msg){}
        virtual void cbGetMapPtr(const std_msgs::UInt64 &msg){ROS_INFO("Planner MAPcb");}
        virtual void cbMapOffset(const geometry_msgs::PointStampedPtr msg){ROS_INFO("Planner MAP Offset cb");}
        virtual void cbCollisionStarting(const std_msgs::Empty){ROS_INFO("Planner Collision Starting cb");}

        virtual void cbGetMutexPtr(const std_msgs::UInt64 &msg){ROS_INFO("Planner Mutexcb");};

        virtual void Setup(){ROS_INFO("Planners Setup");}
        virtual void Run(){ROS_INFO("Planners Run");}

        void ResetWorkspaceSize(StateSpace *stateSpace);
        void PrintWorkspaceSize();
    protected:
        void constructor();
        // ROS Handler
        ros::NodeHandle nh;
        ros::NodeHandle shm_nh;

        // Subscribers
        ros::Subscriber subStop;
        ros::Subscriber subPause;
        ros::Subscriber subStart;
        ros::Subscriber subMapPtr;
        ros::Subscriber subMutexPtr;
        ros::Subscriber subMapOffset;
        ros::Subscriber subWPReached;
        ros::Subscriber subMavrosPose;
        ros::Subscriber subNodeletLoop;
        ros::Subscriber subGoalPosition;
        ros::Subscriber subCollisionPause;
        ros::Subscriber subCollisionStart;
        ros::Subscriber subSafetyDistance;

        // Parameters
        double paramNodeRate;
        double goalFindTolerance;
        double rotationTolerance;

        std::vector<double> robot_size;
        std::vector<double> workspace_size;
        std::vector<double> workspace_center;

        geometry_msgs::Pose goal_position;
        geometry_msgs::PoseStamped varSetPoint;
        geometry_msgs::PoseStamped varRobot;

        std::string map_frame_id;
        std::string robot_frame_id;
        std::string reachedTopic;

        std_msgs::Empty emptyMsg;

        // Publishers
        ros::Publisher pubMarker;
        ros::Publisher pubSetPoint;
        ros::Publisher pubRotSetPoint;
        ros::Publisher pubGoalReached;

        // Variables
        enum PlannerState_t{
            Planning = 0,
            Pause,
            Stop,
            Standby,
            Colliding
        };

        PlannerState_t PlannerState;

        Vector3f mapCenter;
        ros::Time startLastRead;
};

#endif // PLANNER_HPP
