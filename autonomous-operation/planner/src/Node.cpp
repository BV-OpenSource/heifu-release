#include "Planner/Node.hpp"

Node::Node(float x, float y, float z, float theta) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->theta = theta;

    parent = nullptr;
}

Node::Node(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->theta = atan2(this->y, this->x);

    parent = nullptr;
}

Node::Node(geometry_msgs::Point Point) {
    this->x = Point.x;
    this->y = Point.y;
    this->z = Point.z;
    this->theta = atan2(Point.y, Point.x);
    parent = nullptr;
}

Node::~Node(){
    //	printNode("Destruct");
}

geometry_msgs::Point Node::getNodeAsPoint() {
    geometry_msgs::Point p;
    p.x = this->x;
    p.y = this->y;
    p.z = this->z;
    return p;
}

void Node::printNode(std::string name) {
    ROS_INFO(" Node %s: (%.2f, %.2f, %.2f, %.2f) ", name.c_str(), this->x, this->y, this->z, this->theta);
};

bool Node::closeTo(Node *node, double distanceTolerance) {
    return  (this->getSquaredEuclideanDistance(node) < distanceTolerance*distanceTolerance);
}

bool Node::equals(Node *node) {
    return (fabs(this->x - node->x) < DBL_EPSILON) &&
            (fabs(this->y - node->y) < DBL_EPSILON) &&
            (fabs(this->z - node->z) < DBL_EPSILON); //&&
}

void Node::copy(Node *node)
{
    this->x = node->x;
    this->y = node->y;
    this->z = node->z;
    this->parent = node->parent;
    this->children = node->children;
    this->theta = node->theta;
}

double Node::getEuclideanDistance(Node *end){
    float distance = std::numeric_limits<float>::max();
    if(end != nullptr){
        float dx = end->x - this->x;
        float dy = end->y - this->y;
        float dz = end->z - this->z;
        distance = sqrt(dx*dx + dy*dy + dz*dz);
    }
    return distance;
}

double Node::getSquaredEuclideanDistance(Node *end){
    double distanceSquared = std::numeric_limits<double>::max();

    if(end != nullptr){
        double dx = end->x - this->x;
        double dy = end->y - this->y;
        double dz = end->z - this->z;
        distanceSquared = (dx*dx + dy*dy + dz*dz);
    }
    return distanceSquared;
}

void Node::getDeleted()
{
    while(this->children.size()){
        (this->children.front())->getDeleted();
    }
    Node *parent = this->parent;
    if(parent != nullptr){
        parent->children.remove(this);
    }
    delete this;
}

void Node::setAs(float x, float y, float z){
    this->x = x;
    this->y = y;
    this->z = z;
    this->theta = atan2(this->y, this->x);
}

void Node::setAs(geometry_msgs::Point Point){
    this->x = Point.x;
    this->y = Point.y;
    this->z = Point.z;
    this->theta = atan2(Point.y, Point.x);
}
