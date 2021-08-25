#ifndef PLANNERS_STATESPACE_HPP
#define PLANNERS_STATESPACE_HPP

#include "Planner/Node.hpp"
#include "Planner/Obstacle.hpp"
#include <tf/tf.h>

class StateSpace {
    public:
        // defines the state space
        StateSpace(std::vector<double> center, std::vector<double> size, double angle = 0);

        // generates a random c-space configuration in the state space
        Node * genRandomNodeInSpace();

        // check if a state configuration is obstructed
        bool isObstructed(Node *node, bool ignoreSafety = false);
        bool isGoalObstructed(Node *node, const double margin);

        // add an obstacle to the statespace
        void addObstacle(Obstacle *obstacle);
        void updateObstacleMapCenter(Vector3f mapCenter);
        void updateObstacleSafetyDistance(double newDistance);
        void cleanObstacles();

        bool edgeIsObstructed(Node *nearestNode, Node *newNode, bool ignoreSafety = false);

        std::vector<Node *> smoothenPath(std::vector<Node *> roughShortestPath);

        void setMaxMin(std::vector<double> workspace_center, std::vector<double> workspace_size, std::vector<double> centerOffset={0,0,0}, double angle = 0);

        //check if the node is within the statespace
        bool isWithinStateSpace(Node *node);

        // If Goal, statespace is smaller
        bool isGoalWithinStateSpace(Node *node, const double margin);

        bool isEdgeGoalObstructed(Node *init, Node *goal, double margin, bool ignoreSafety);

        Node *getClosestPoint(Node *goalNode, const double margin, uint8_t &planeAxis);
        Node *getFreeGoalPoint(Node *goalNode, const double margin);

        geometry_msgs::Point getCenter();
        geometry_msgs::Point getSize();
        geometry_msgs::Point getGeometricCenter();

        bool nodeInSpace(Node *leaf);

        geometry_msgs::Point getRotatedVectorToCenter(Node *goalNode, double angle);

        Node *poseUAVNode = nullptr;
    private:
        geometry_msgs::Point _center;
        geometry_msgs::Point _size;

        double min_x, max_x, min_y, max_y, min_z, max_z;

        double rotation;

        std::list<Obstacle *> obstacles;

        //generate random double
        double generateRandomNumber(double min, double max);
};

#endif //PLANNERS_STATESPACE_HPP
