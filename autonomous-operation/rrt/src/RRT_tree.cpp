#include "RRT/RRT_tree.h"

namespace RRT3D {

    RRT_Tree::RRT_Tree() {
        root = nullptr;
    };

    void RRT_Tree::insert(Node *newNode, Node *leaf) {
        if (root != nullptr && leaf != nullptr) {
            leaf->children.push_back(newNode);
            newNode->parent = leaf;
        } else {
            root = newNode;
        }
    }

    void RRT_Tree::printTree(){
        printTreeAfterNode(root);
    }

    void RRT_Tree::printTreeAfterNode(Node *leaf) {
        if (leaf == nullptr) {
            return;
        } else {
            leaf->printNode("");
            for (std::list<Node *>::iterator it = leaf->children.begin(); it != leaf->children.end(); it++) {
                printTreeAfterNode(*it);
            }
        }
    }

    void RRT_Tree::printChilds(Node *leaf) {
        if (leaf == nullptr) {
            return;
        } else {
            leaf->printNode("Parent");
            for (std::list<Node *>::iterator it = leaf->children.begin(); it != leaf->children.end(); it++) {
                (*it)->printNode("Child");
            }
        }
    }

    Node * RRT_Tree::getNearestNeighbor(Node *randomNode, StateSpace *stateSpace, double &minDistance) {
        Node *closestNode = nullptr;

        if (root != nullptr && randomNode != nullptr) {
            minDistance = std::numeric_limits<double>::max();
            std::vector<Node *> treeNodes;
            treeNodes.push_back(root);

            /*
             * Calculate the euclidean distance for each node in the tree and get the min
             * This is done in a breadth-first manner.
            */
            Node *currentNode = nullptr;
            double currentNodeDistance = minDistance;
            while (!treeNodes.empty()) {
                currentNode = treeNodes.front();
                currentNodeDistance = currentNode->getEuclideanDistance(randomNode);
                if(stateSpace->isWithinStateSpace(currentNode)){
                    if (minDistance > currentNodeDistance) {
                        minDistance = currentNodeDistance;
                        closestNode = currentNode;
                    }
                }
                // insert the children of that node to the search list
                treeNodes.insert(treeNodes.end(), currentNode->children.begin(), currentNode->children.end());
                // delete the processed node
                treeNodes.erase(treeNodes.begin());
            }
        }
        return closestNode;
    }

    Node * RRT_Tree::extendNode(Node *currentNode, Node *randomNode, double epsilon) {
        double deltaX = randomNode->x - currentNode->x;
        double deltaY = randomNode->y - currentNode->y;
        double deltaZ = randomNode->z - currentNode->z;
        double x = 0, y = 0, z = 0;

        double VectMag = sqrt(deltaX*deltaX+deltaY*deltaY+deltaZ*deltaZ);
        double EpsilonCoef = epsilon/VectMag;

        x = currentNode->x + deltaX*EpsilonCoef;
        y = currentNode->y + deltaY*EpsilonCoef;
        z = currentNode->z + deltaZ*EpsilonCoef;

        return new Node(x, y, z);
    }

    std::vector<Node *> RRT_Tree::extractFullPath(Node *goal, Node *start, StateSpace *stateSpace){
        std::vector<Node *> shortestPathFromGoal;
        if(goal == nullptr){
            ROS_ERROR("GOAL NOT DEFINED");
            shortestPathFromGoal.clear();
            return shortestPathFromGoal;
        }
        if(start == nullptr){
            ROS_ERROR("START NOT DEFINED");
            shortestPathFromGoal.clear();
            return shortestPathFromGoal;
        }

        shortestPathFromGoal.push_back(goal);

        Node *parentNode = goal->parent;
        if(parentNode == nullptr){
            ROS_ERROR("Bad insert");
            shortestPathFromGoal.clear();
            return shortestPathFromGoal;
        }

        while (!parentNode->closeTo(start, 1.0)) {
            if(!stateSpace->isObstructed(parentNode) || (!stateSpace->isObstructed(parentNode, true) && parentNode->closeTo(start, 2.0))){
                shortestPathFromGoal.push_back(parentNode);
            }else{
                shortestPathFromGoal.clear();
                parentNode->printNode("Is obstructed");
                return shortestPathFromGoal;
            }

            if(!parentNode->equals(root) && parentNode->parent != nullptr){
                parentNode = parentNode->parent;
            }else{
                ROS_WARN("\n\tReached the root of the tree and couldn't find a node near robot.");
                shortestPathFromGoal.clear();
                return shortestPathFromGoal;
            }
        }

        if(shortestPathFromGoal.size() > 0){
            shortestPathFromGoal.push_back(start);
        }
        //		std::cout<< "printing the NodePath\n";
        //		for (int i = 0; i < shortestPathFromGoal.size(); ++i) {
        //			shortestPathFromGoal.at(i)->printNode("Path");
        //		}
        return shortestPathFromGoal;
    }

    std::vector<Node *> RRT_Tree::processFinalRobotPath(std::vector<Node *> path) {
        std::vector<Node *> finalPath;
        Node *currentNode = path.back();
        finalPath.push_back(currentNode);
        //		Node *parentNode = path.back()->parent;
        //		while (!parentNode->equals(root)) {
        //			parentNode->calculateGradient(currentNode);
        //			finalPath.push_back(parentNode);
        //			currentNode = parentNode;
        //			parentNode = parentNode->parent;
        //		}
        //		parentNode->calculateGradient(currentNode);
        //		finalPath.push_back(parentNode);
        //		ROS_INFO("%zu ROBOT PATH SIZE",finalPath.size());
        return finalPath;
    }

    Node *RRT_Tree::GetRoot(){
        return this->root;
    }

    void RRT_Tree::SetRoot(Node *newRoot){
        this->root = newRoot;
    }
}
