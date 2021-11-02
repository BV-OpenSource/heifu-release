#ifndef PLANNERS_NODE_HPP
#define PLANNERS_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

class Node {
    public:
        Node(float x, float y, float z, float theta);
        Node(float x, float y, float z);
        Node(geometry_msgs::Point);

        ~Node();

        float x, y, z, theta;

        Node *parent;

        std::list<Node *> children;

        geometry_msgs::Point getNodeAsPoint();

        void printNode(std::string name);
        bool closeTo(Node *node, double distanceTolerance);
        bool equals(Node* node);

        void copy(Node *node);
        double getEuclideanDistance(Node *end);
        double getSquaredEuclideanDistance(Node *end);

        void getDeleted();

        void setAs(float x, float y, float z);
        void setAs(geometry_msgs::Point Point);
};

#endif //PLANNERS_NODE_HPP
